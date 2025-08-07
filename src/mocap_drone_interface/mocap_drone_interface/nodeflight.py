from mocap_drone_interface.library.NatNetClient import NatNetClient
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Point
from std_msgs.msg import String, Header
import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from cflib.positioning.motion_commander import  MotionCommander
import time
import datetime


class MocapPublisherNode(Node):
    def __init__(self, local_ip, server_ip, ids: list):
        self.local_ip = local_ip
        self.server_ip = server_ip
        self.ids = set(ids)
        
        self.init()

    def init(self):
        super().__init__('pose_publisher')
        self.publishers_ = {}
        self.posepub_ = {}
        
        
        for i in self.ids:
            topic1 = f'/drone{i}/posestamped'
            self.publishers_[i]  = self.create_publisher(PoseStamped, topic1, 10)
            topic2 = f'drone{i}/pose'
            self.posepub_[i] = self.create_publisher(Pose, topic2, 10)
        self.client = NatNetClient()
        self.client.set_use_multicast(False)
        self.client.set_client_address(self.local_ip)
        self.client.set_server_address(self.server_ip)
        self.client.rigid_body_listener = self.rrbf
        self.client.set_print_level(0)
        self.counter = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.main = threading.Thread(target=self.main_thread)
        self.main.start()
        
        
        

    def rrbf(self, new_id, position, rotation):
    
        if not rclpy.ok():
            return 
        if new_id in self.ids:
            msg=PoseStamped()
            msg2 = Pose()
            x, y, z = position
            qx, qy, qz, qw = rotation
            #msg.data = f"{new_id},{x:.3f},{y:.3f},{z:.3f},{qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}"
            msg2.position.x = x
            msg2.position.y = y
            msg2.position.z = z
            msg2.orientation.x = qx
            msg2.orientation.y = qy
            msg2.orientation.z = qz
            msg2.orientation.w = qw

            self.counter += 1
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
    
            with self.lock:
                self.publishers_[new_id].publish(msg)
                self.posepub_[new_id].publish(msg2)
                print(msg2)
                
        
    def main_thread(self):
        while True:
            self.client.run('d')

            time.sleep(1)
            if self.client.connected():
                print("Connected to server")
                print("Dtarting data feed")
                break
            else:
                print("Not connected")
        while not self.stop_event.is_set():
            time.sleep(0.01)
    
        print("Main thread exiting")
        
        
        
    
    def callshutdown(self):
        self.stop_event.set()
        self.client.shutdown()
        self.main.join()
        print("disconnected")
        print("All done. Active threads:")
        
        for t in threading.enumerate():
            print(f"- {t.name} (daemon={t.daemon})")
    
class MocapCaller():
    def __init__(self, local_ip, server_ip, ids):
        rclpy.init()
        publisher_ = MocapPublisherNode(local_ip, server_ip, ids)
        try:
            rclpy.spin(publisher_)
        except KeyboardInterrupt:
            print("KeyboardInterrupt received, shutting down...")
        finally:
            publisher_.stop_event.set()
            
            publisher_.client.shutdown()
            
            publisher_.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

class ControlPublisher(Node):
    def __init__(self, id):
        self.id = id
        name = f'drone{id}_controller'
        super.__init__(name)
        topic = f'drone{id}/control'
        publisher_ = self.create_publisher(Point, topic, 10)

