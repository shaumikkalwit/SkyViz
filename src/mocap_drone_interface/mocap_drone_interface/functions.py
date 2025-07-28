import sys
import time
import csv
import datetime
import os
#try:
from library.NatNetClient import NatNetClient  # or whatever module you're using
#except ImportError:
    #print("Please download the NatNet SDK and place the Python client in the proper directory.")
    #exit(1)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


import threading
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from cflib.positioning.motion_commander import  MotionCommander





class control():
    def __init__(self, id):
        self.id  = id
        self.lock = threading.Lock()
    def rrbf(self, new_id, position, rotation):
        global newpos
        global newrot
        if new_id == self.id:
            newpos = position
            newrot = rotation
        else: 
            pass

    def mocap_listener(self, local_ip, stop_event: threading.Event, cf: Crazyflie, server_ip: str = '10.131.196.172'):
        global newpos
        global newrot
        
        # local_ip and server_ip should be used as strings

        self.client = NatNetClient()
        self.client.set_use_multicast(False)
        self.client.set_client_address(local_ip)
        self.client.set_server_address(server_ip)
        self.client.rigid_body_listener = self.rrbf
        self.client.set_print_level(0)
        self.counter = 0
        
        while True:
            self.client.run('d')

            time.sleep(1)
            if self.client.connected():
                print("Connected to server")
                break
            else:
                print("Not connected")
        
        while not stop_event.is_set():
            with self.lock:
                if newpos is not None and newrot is not None:
                    x, y, z = newpos
                    qx, qy, qz, qw = newrot
                    cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
                    self.counter +=1 
                    #if self.counter%50 == 0:
                        
                        #print(f"Sending extpos: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                time.sleep(0.02)
        
        self.client.shutdown()

                    # 10hz data send rate

    def init_drone(self, cf: Crazyflie):
        
        cf.platform.send_arming_request(True)
        print("Crazyflie armed")
        cf.param.set_value('stabilizer.estimator', '2')  # Kalman
        cf.param.set_value('commander.enHighLevel', '1')
        reset_estimator(cf)
        time.sleep(5)
        print('values set')
       

    def takeoff(self, cf: SyncCrazyflie):
        global newpos
        global newrot
        with MotionCommander(cf, 0.5) as mc:

           
           time.sleep(5)
           mc.land()
           time.sleep(5)

 

        

        # while newpos is None or newrot is None:
        #     time.sleep(1)
        #     print("Waiting for pose data")
        # for _ in range(100):
        #     print("this would be a takeoff attempt")
        #     time.sleep(0.1)

class mocap_basic_pub(Node):
    def __init__(self, group, local, server):
        #initialize the ip addresses 
        
        self.localip = local
        self.serverip = server
        self.groupid = group
        
    
        
        self.counter = 0
        self.client = NatNetClient()
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', group)


        self.mocap_init()
        self.running = True
      
    def rrbf(self, new_id, position, rotation):
        
        if not rclpy.ok():
            return
        # Format as "id;x,y,z;qx,qy,qz,qw"
        pos_str = ",".join(f"{x:.3f}" for x in position)
        rot_str = ",".join(f"{x:.4f}" for x in rotation)
        msg_str = f"{new_id};{pos_str};{rot_str}"
        msg = String()
        msg.data = msg_str

        print(f"Publishing ID {new_id}: {msg.data}")  # DEBUG

        try:
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Publish failed: {e}")
            return

        
        self.counter += 1
        
        if self.counter % 10 == 0:
            pass
            #print(f"\rFrame {self.counter} Time {time:.2f}", end='', flush=True)
            #self.get_logger().info(f"\rFrame {self.counter} Time {time} Position: {self.position.x}, {self.position.y}, {self.position.z}", end='',flush=True)

    def mocap_init(self):

        # init mocap
        
        self.client.set_use_multicast(False)
        self.client.set_client_address(self.localip)
        self.client.set_server_address(self.serverip)
        self.client.rigid_body_listener = self.rrbf
        self.client.set_print_level(0)

        # connect loop until it connects
        while True:
            self.client.run('d')

            time.sleep(1)
            if self.client.connected():
                
                break
            else:
                print("Not connected")
    
    def destroy_node(self):
        self.running = False
        self.client.shutdown()
        super().destroy_node()
                
class mocap_basic_sub(Node):
    
    def __init__(self, group):
        self.groupid = group
        
        
        
        super().__init__('minimal_subscriber')

        self.declare_parameter('id', 0)
        id_param = self.get_parameter('id').get_parameter_value().integer_value
        self.streamid = str(id_param)
        
        self.declare_parameter('uri', 'E7E7E7E7E7')
        uri_param = self.get_parameter('uri').get_parameter_value().string_value
        self.uri = uri_param

        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, group)

    def listener_callback(self, msg):
        data = msg.data  # e.g. "5;1.234,2.345,3.456;0.0000,0.0000,0.0000,1.0000"
        
        # Split the string by ';' into parts: [id, position_str, rotation_str]
        parts = data.split(';')
        if len(parts) != 3:
            self.get_logger().warning(f"Unexpected data format: {data}")
            return
        
        received_id = parts[0]
        position_str = parts[1]  # e.g. "1.234,2.345,3.456"
        rotation_str = parts[2]  # e.g. "0.0000,0.0000,0.0000,1.0000"

        if received_id == self.streamid:
            # Further split position and rotation by ','
            position = [float(x) for x in position_str.split(',')]  # [1.234, 2.345, 3.456]
            rotation = [float(x) for x in rotation_str.split(',')]  # [0.0, 0.0, 0.0, 1.0]
        
            # Now you can use received_id, position, and rotation as needed
        
            print(f"ID: {received_id}")
            print(f"Position: {position}")
            print(f"Rotation: {rotation}")

class complete_node_pub(mocap_basic_pub):
    def __init__(self, group = 10, local = "10.131.220.228", server = "10.131.196.172"):
        rclpy.init()

        publisher = mocap_basic_pub(group, local, server)
        try: 
            rclpy.spin(publisher)
        except KeyboardInterrupt:
            pass
        finally:
            publisher.destroy_node()

            rclpy.shutdown()

class complete_node_sub(mocap_basic_sub):
    def __init__(self, group = 10):
        rclpy.init()
        
        
        subscriber = mocap_basic_sub(group)
        try: 
            rclpy.spin(subscriber)
        except KeyboardInterrupt:
            pass
        finally:
            subscriber.destroy_node()

            rclpy.shutdown()

class mocap_communicator_node(Node):
    def __init__(self, group):
        self.group=group
        super().__init__('mocap_communicator_node')
        self.declare_parameter('id', 0)
        id_param = self.get_parameter('id').get_parameter_value().integer_value
        self.streamid = str(id_param)
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, self.group)

    def listener_callback(self, msg):
        data = msg.data

        # Split the string by ';' into parts: [id, position_str, rotation_str]
        parts = data.split(';')
        if len(parts) != 3:
            self.get_logger().warning(f"Unexpected data format: {data}")
            return
        
        received_id = parts[0]
        position_str = parts[1]  
        rotation_str = parts[2]  

        if received_id == self.streamid:
            # Further split position and rotation by ','
            position = [float(x) for x in position_str.split(',')]  
            rotation = [float(x) for x in rotation_str.split(',')]  

class sc_flight():
    def __init__(self, local: str, server: str, id: int, uri):
        self.lock = threading.Lock()
        cflib.crtp.init_drivers()
        print(f"drivers initiated for drone {id}")
        self.localip = local
        self.serverip = server
        self.uri = uri_helper.uri_from_env(uri)
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
        self.scf.__enter__()
        self.cf = self.scf.cf
        
        self.id = id
        
        self.stop_event = threading.Event()
        self.mocap = threading.Thread(target=self.mocap_listener, args=(local,  self.stop_event, self.cf,server))
        self.mocap.start()

        self.init_drone(self.cf)
        self.commander = self.cf.high_level_commander
        time.sleep(1)

    def init_drone(self, cf: Crazyflie):
        
        cf.platform.send_arming_request(True)
        print("Crazyflie armed")
        cf.param.set_value('stabilizer.estimator', '2')  # Kalman
        cf.param.set_value('commander.enHighLevel', '1')
        reset_estimator(cf)
        time.sleep(5)
        print('values set')
       
    def rrbf(self, new_id, position, rotation):
        
        if new_id == self.id:
            self.newpos = position
            self.newrot = rotation
        else: 
            pass

    def mocap_listener(self, local_ip, stop_event: threading.Event, cf: Crazyflie, server_ip: str = '10.131.196.172'):
        
        
        # local_ip and server_ip should be used as strings

        self.client = NatNetClient()
        self.client.set_use_multicast(False)
        self.client.set_client_address(local_ip)
        self.client.set_server_address(server_ip)
        self.client.rigid_body_listener = self.rrbf
        self.client.set_print_level(0)
        self.counter = 0
        
        while True:
            self.client.run('d')

            time.sleep(1)
            if self.client.connected():
                print("Connected to server")
                break
            else:
                print("Not connected")
        
        while not stop_event.is_set():
            with self.lock:
                if self.newpos is not None and self.newrot is not None:
                    x, y, z = self.newpos
                    qx, qy, qz, qw = self.newrot
                    cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
                    self.counter +=1 
                    if self.counter%50 == 0:
                        
                        print(f"Sending extpos for id: {self.id}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                time.sleep(0.02)
        
        self.client.shutdown()

                    # 10hz data send rate

    
    def logger_thread(self, rr: int):
        """
        CSV Logger Method logs pose data into a csv file. 
        
        Args:
            rr (int): Data write rate in HZ

        """
        package_root = os.path.dirname(os.path.abspath(__file__))  
        log_dir = os.path.join(package_root, '..', 'logs')        
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.datetime.now().strftime("log_%Y%m%d_%H%M%S.csv")
        filepath = os.path.join(log_dir, filename)
        
        self.csvfile = open(filepath, 'a', newline='')
        csvwriter = csv.writer(self.csvfile)
        
        
        duration = 1/rr
        try:
            while not self.stop_event.is_set():
                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                x, y, z = self.newpos
                x_rounded = round(x, 3)
                y_rounded = round(y, 3)
                z_rounded = round(z, 3)
                data_row = [timestamp, x_rounded, y_rounded, z_rounded]
                csvwriter.writerow(data_row)
                self.csvfile.flush() 
                #print(f"Data logged: {data_row}")
                time.sleep(duration)
        finally:
            self.csvfile.close()

    def csv_logger(self, rr: int):
        self.loggerthread = threading.Thread(target=self.logger_thread, args=(rr,))
        self.loggerthread.start()
        
    def takeoff(self, height, sec):
        
        self.commander.takeoff(height, sec)
        print(f"Taking off to {height} meters over {sec} seconds.")
        time.sleep(sec)
    
    def go_to(self, x, y, z, yaw, sec):
        self.commander.go_to(x, y, z, yaw, sec)
        print(f"Moving to {x}, {y}, {z} over {sec} seconds.")
        time.sleep(sec)
    
    def land(self, height, sec):
        self.commander.land(height, sec)
        print(f"Landing to {height} meters over {sec} seconds.")
        time.sleep(sec)


    def disconnect(self):
        if self.scf:
            
            self.stop_event.set()
            self.mocap.join()
            if hasattr(self, 'loggerthread') and self.loggerthread:
                self.loggerthread.join()
            self.scf.__exit__(None, None, None)

            print("disconnected")
            print("All done. Active threads:")
            for t in threading.enumerate():
                print(f"- {t.name} (daemon={t.daemon})")

class rviz_connector(Node):
    def __init__(self, id, local, server):
        super().__init__('minimal_publisher')
        topic_name = f'/drone{id}/pose'
        self.publisher_ = self.create_publisher(PoseStamped, topic_name, 10)
        
        self.client = NatNetClient()
        self.localip =  local
        self.serverip = server
        self.id = id
        self.counter = 0
        self.mocap = threading.Thread(target=self.mocap_start)
        self.stop_event = threading.Event()
        self.mocap.start()
        

    def rrbf(self, new_id, position, rotation):       
        if new_id == self.id:
            self.newpos = position
            self.newrot = rotation
            msg = PoseStamped()
            self.counter += 1
            msg.header.frame_id = str(self.counter)
            msg.pose.position = Point()
            pos = msg.pose.position 
            pos.x, pos.y, pos.z = self.newpos
            msg.pose.orientation = Quaternion()
            quat = msg.pose.orientation
            quat.x, quat.y, quat.z, quat.w = self.newrot
            self.publisher_.publish(msg)
        else: 
            pass

    def mocap_start(self):
        self.client.set_use_multicast(False)
        self.client.set_client_address(self.localip)
        self.client.set_server_address(self.serverip)
        self.client.rigid_body_listener = self.rrbf
        self.client.set_print_level(0)

        # connect loop until it connects
        while True:
            self.client.run('d')

            time.sleep(1)
            if self.client.connected():
                
                break
            else:
                print("Not connected")

    def disconnect(self):
        self.stop_event.set()
        self.mocap.join()
        
        

        print("disconnected")
        print("All done. Active threads:")
        for t in threading.enumerate():
            print(f"- {t.name} (daemon={t.daemon})")

class rviz_node(rviz_connector):
    def __init__(self, id = 10, local = "10.131.220.228", server = "10.131.196.172"):
        rclpy.init()

        publisher = rviz_connector(id, local, server)
        try: 
            rclpy.spin(publisher)
        except KeyboardInterrupt:
            pass
        finally:
            publisher.disconnect()
            publisher.destroy_node()

            rclpy.shutdown()