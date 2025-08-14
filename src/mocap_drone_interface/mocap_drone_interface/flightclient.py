import rclpy
from drone_viz_interfaces.srv import FlightService
from drone_viz_interfaces.msg import Command
import time
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from geometry_msgs.msg import Pose, PoseStamped, Point
from cflib.utils.reset_estimator import reset_estimator

from mocap_drone_interface.library.NatNetClient import NatNetClient
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from typing import Dict
import yaml, os
from ament_index_python.packages import get_package_share_directory

class MocapGrabberClass():
    def __init__(self, ids: list[int], uris: list[str], node: Node, server: str, local: str):
        self.publishers_ = {}
        self.ids = ids
        for id in self.ids:
            self.topic_string = f'/drone{id}/posestamped'
            self.publishers_[id] = node.create_publisher(PoseStamped, self.topic_string, 10)
        self.client = NatNetClient()
        self.client.set_use_multicast(False)
        self.client.set_client_address(local)
        self.client.set_server_address(server)
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
            
            x, y, z = position
            qx, qy, qz, qw = rotation
            
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
                

class MinFlightService(Node):   
    def __init__(self, cf: Crazyflie):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.runningstate = False
        super().__init__('minimal_flight_service')

        self.declare_parameter('data.localip', '127.0.0.1')
        self.declare_parameter('data.serverip', '127.0.0.1')
        self.declare_parameter('data.ids', [1, 2, 3])
        self.declare_parameter('data.uris',['E7E7E7E7E7', 'E7E7E7E7E8', 'E7E7E7E7E9'])
        self.local_ip = self.get_parameter('data.localip').get_parameter_value().string_value
        self.server_ip = self.get_parameter('data.serverip').get_parameter_value().string_value
        self.ids = set(self.get_parameter('data.ids').get_parameter_value().integer_array_value)
        self.uris = self.get_parameter('data.uris').get_parameter_value().string_array_value

        self.positionlock = threading.Lock()
        self.statelock = threading.Lock()
        self.srv = self.create_service(FlightService, 'flight_service', self.flight_service_callback)
        subscriber_ = self.create_subscription(Pose,'/drone11/pose', self.PoseCallback, 10)
        
        self.cf = cf
        self.arm_state = False
        self.mc = self.cf.high_level_commander
        threading.Thread(target=self.pose_loop, daemon=True).start()
        print("Thread started")

    def pose_loop(self):
        while rclpy.ok():
            with self.positionlock:
                pose = (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
            self.cf.extpos.send_extpose(*pose)
            # print(f"sent pose: {pose}")
            time.sleep(0.05)

        
    def PoseCallback(self, msg:Pose):
        
        with self.positionlock:
                self.x = msg.position.x
                self.y = msg.position.y
                self.z = msg.position.z
                self.qx = msg.orientation.x
                self.qy = msg.orientation.y
                self.qz = msg.orientation.z
                self.qw = msg.orientation.w
                
                
    def flight_service_callback(self, request: FlightService.Request, response: FlightService.Response):
        
        if request.command_input.comtype == 'a':
            print("Request received, arming drone!")
            with self.statelock:
                if not self.runningstate:
                    threading.Thread(target=self.arm_and_setup, args=(request, response)).start()
                    
                    response.success = True
                    response.message = "Arm request sent correctly"
                    return response
        elif request.command_input.comtype == 't':
            print("Request received, taking off!")
            if not self.arm_state:
                response.success = False
                response.message = "Drone not armed yet!"
                return response
            with self.statelock:
                if not self.runningstate:
                    threading.Thread(target=self.takeoff, args=(request, response)).start()
                    response.success = True
                    response.message = "Takeoff request sent correctly"
                    return response
                else: 
                    response.success = False
                    response.message = "Request in progress already"
                    return response
        elif request.command_input.comtype == 'm':
            print("Request received, moving drone!")
            if not self.arm_state:
                response.success = False
                response.message = "Not armed yet!"
                return response
            with self.statelock:
                if not self.runningstate:
                    threading.Thread(target=self.moveto, args=(request, response)).start()
                    response.success = True
                    point = request.command_input.moveto
                    response.message = f"Move request sent for {point.x}, {point.y}, {point.z}"
                    return response
        elif request.command_input.comtype == 'l':
            print("Request received, landing drone!")
            if not self.arm_state:
                response.success = False
                response.message = "Drone not armed yet!"
                return response
            with self.statelock:
                if not self.runningstate:
                    threading.Thread(target=self.land, args=(request, response)).start()
                    response.success = True
                    response.message = "land request sent correctly"
                    return response
                else: 
                    response.success = False
                    response.message = "Request in progress already"
                    return response
    
    def arm_and_setup(self, request: FlightService.Request, response: FlightService.Response):
        with self.statelock:
            self.runningstate = True
        self.get_logger().info("Arming Crazyflie")
        self.cf.platform.send_arming_request(True)
        time.sleep(1)
        self.cf.param.set_value('stabilizer.estimator', '2')  # Kalman
        time.sleep(0.1)
        
        
        self.arm_state = True
        reset_estimator(self.cf)
        time.sleep(5)
        

        with self.statelock:
            self.runningstate = False
        print("drone armed and ready", flush=True)
    
    def takeoff(self, request: FlightService.Request, response: FlightService.Response):    
        with self.statelock:
            self.runningstate = True
        self.mc.takeoff(1,5)
        time.sleep(5)
        with self.statelock:
            self.runningstate = False
        print("takeoff complete")
    
    def moveto(self,request: FlightService.Request, response: FlightService.Response):
        with self.statelock:
            self.runningstate = True
        x = request.command_input.moveto.x
        y = request.command_input.moveto.y
        z = request.command_input.moveto.z
        if request.command_input.absolute == False:
            self.mc.go_to(x, y, z, 0, 4, True)
            print(f"Moving to relative position: {x}, {y}, {z}")
        else:
            self.mc.go_to(x,y,z,0,4)    
            print(f"Moving to absolute position: {x}, {y}, {z}")
        with self.statelock:
            self.runningstate = False
    
    def land(self, request: FlightService.Request, response: FlightService.Response):
        with self.statelock:
            self.runningstate = True
        self.mc.land(0,5)
        time.sleep(5)
        with self.statelock:
            self.runningstate = False
      

class MultiDroneFlightService(Node):
    def __init__(self):
        # take all ros parameters in
        super().__init__('MDFS_Node')

        self.declare_parameter('data.localip', '127.0.0.1')
        self.declare_parameter('data.serverip', '127.0.0.1')
        self.local_ip = self.get_parameter('data.localip').get_parameter_value().string_value
        self.server_ip = self.get_parameter('data.serverip').get_parameter_value().string_value
        pkg_path = get_package_share_directory('mocap_drone_interface')
        yaml_file = os.path.join(pkg_path, 'config', 'params.yaml')
        
        with open(yaml_file, 'r') as f:
            config = yaml.safe_load(f)

        drones: dict = config['/**']['ros__parameters']['drones']
        print(drones)
        self.ids = []

        self.uris = []

        

        # running state flag to aid in mocap shutdown
        self.runningstate = True
        

        
        self.statelock = threading.Lock()

        # initialize motion capture parameters
        self.client = NatNetClient()
        self.client.set_use_multicast(False)
        self.client.set_client_address(self.local_ip)
        self.client.set_server_address(self.server_ip)
        self.client.rigid_body_listener = self.rrbf
        self.client.set_print_level(0)
        self.lock = threading.Lock()
        


        
        
        # create a single service overall and then initialize separate classes of control for each individual drone
        self.service = self.create_service(FlightService, 'MDFS_Service', self.fscallback)
        
        
        
    
        # initialize classes for each drones control

        self.controllers: Dict[int, DroneController] = {}
        for name, attrs in drones.items():
            drone_id = int(attrs['id'])
            print(drone_id)
            self.controllers[drone_id] = DroneController(
                drone_id,
                uri=attrs['uri']
                )
            self.ids.append(drone_id)
        print(self.ids, flush=True)
        print(f"Created {len(self.controllers)} drone objects.", flush=True)
        print(self.controllers)

        # create publishers for each drone passed in IDS

        self.publishers_ = {}

        for id in self.ids:
            topicname = f'/drone{id}/posestamped'
            self.publishers_[id] = self.create_publisher(PoseStamped, topicname, 10)
            print(id)
        print(self.publishers_)

        self.mocap = threading.Thread(target=self.mocap_functionality)
        self.mocap.start()
        print("mocap thread started", flush=True)
            
        

    def rrbf(self, new_id, position, rotation):
        print(f"Publishing pose for drone {new_id}", flush=True)
        if new_id in self.ids:
            
            if new_id not in self.controllers:
                return
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'

            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = rotation            
            drone = self.controllers[new_id]
            drone.x, drone.y, drone.z = position
            drone.qx, drone.qy, drone.qz, drone.qw = rotation
            if rclpy.ok():
                self.publishers_[new_id].publish(msg)

    def mocap_functionality(self):
        while True:
            self.client.run('d')

            time.sleep(1)
            if self.client.connected():
                print("Connected to server")
                break
            else:
                print("Not connected")
               
        while self.runningstate:
            time.sleep(1)
        

    def fscallback(self, request: FlightService.Request, response: FlightService.Response):
        droneid = request.command_input.droneid
        if droneid in self.ids:
            if self.controllers[droneid].activerequest == False :
                threading.Thread(target=self.controllers[droneid].request, args=(request,)).start()
                response.success = True
                response.message = "Command sent safely. Processing now..."
            elif request.command_input.override:
                self.get_logger().warning("Overriding current command, continuing")
                threading.Thread(target=self.controllers[droneid].request, args=(request,)).start()
                response.success = True
                response.message = "Command sent with override. Processing now..."
            else:
                self.get_logger().warning("Drone is already executing a command")
                response.success = False
                response.message = f"Failed to process command, drone {droneid} is busy."
        else: 
            self.get_logger().error("Command called on nonexistent drone. ")
            response.success = False
            response.message = f"Drone {droneid} does not exist."
        return response

    def disconnect(self):
        for id in self.ids:
            self.controllers[id].shutdown()
        self.runningstate = False
        self.client.stop_threads = True 
        self.mocap.join(timeout=2)  
        print("disconnected")
        
        
class DroneController():
    def __init__(self, id, uri): 
        # initialize position variables as 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.qw = 0

        self.runstate = True
        self.activerequest = False
    
        self.id = id
        self.uri = 'radio://0/80/2M/' + uri
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache'))
        self.scf.open_link()
    
        self.hlc = self.scf.cf.high_level_commander

        
        print(f"Drone initialized with ID {id} and URI {self.uri}", flush=True)
        
        
        
        self.posepub = threading.Thread(target=self.pose_updater, daemon=True)
        self.posepub.start()
        
        

    def pose_updater(self):
        while self.runstate:
            self.scf.cf.extpos.send_extpose(self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
            time.sleep(0.01)
    
    def request(self, request: FlightService.Request):
        response = FlightService.Response()
        self.activerequest = True
        print("request received and processing", flush=True)
        goalpoint = request.command_input.moveto
        if request.command_input.comtype == 'a':
            self.arm()
        elif request.command_input.comtype == 't':
            self.takeoff()
        elif request.command_input.comtype == 'l':
            self.land()
        elif request.command_input.comtype == 'm':
            self.moveto(goalpoint, request.command_input.absolute)
        self.activerequest = False
        time.sleep(1)
        print("request completed processing", flush=True)

    def arm(self):
        self.scf.cf.platform.send_arming_request(True)
        time.sleep(1)
        self.scf.cf.param.set_value('stabilizer.estimator', '2')  # Kalman
        time.sleep(0.1)
        
        
        self.arm_state = True
        reset_estimator(self.scf.cf)
        time.sleep(5)
        print(f"Drone {self.id} armed.")

    def takeoff(self):
        self.hlc.takeoff(1, 5)

    def land(self):
        self.hlc.land(0, 5)
    
    def moveto(self, point: Point, abs: bool):
        if abs:
            self.hlc.go_to(point.x, point.y, point.z, 0, 3, False)
        else: 
            self.hlc.go_to(point.x, point.y, point.z, 0, 3, True)
        

    def shutdown(self):
        self.runstate = False
        self.posepub.join()
        self.scf.close_link()
        print(f"Drone instance {self.id} killed")
        


        
def main():
    rclpy.init()
    cflib.crtp.init_drivers()
    mdfs = MultiDroneFlightService()
    
    

    try:
        mdfs.get_logger().info("MultiDroneFlightService is now spinning and ready.")
        rclpy.spin(mdfs)
        
    except KeyboardInterrupt:
        mdfs.disconnect()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
  