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
from geometry_msgs.msg import Pose
from cflib.utils.reset_estimator import reset_estimator



class MinFlightService(Node):
    def __init__(self, cf: Crazyflie):
        self.runningstate = False
        super().__init__('minimal_flight_service')
        self.positionlock = threading.Lock()
        self.statelock = threading.Lock()
        self.srv = self.create_service(FlightService, 'flight_service', self.flight_service_callback)
        subscriber_ = self.create_subscription(Pose,'/drone11/pose', self.PoseCallback, 10)
        
        self.cf = cf
        self.arm_state = False
        self.mc = self.cf.high_level_commander
        threading.Thread(target=self.pose_loop, daemon=True).start()

    def pose_loop(self):
        while rclpy.ok():
            with self.positionlock:
                pose = (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
            self.cf.extpos.send_extpose(*pose)
            print(f"sent pose: {pose}")
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
        print(f"request received: {request}")
        if request.command_input.comtype == 'a':
            with self.statelock:
                if not self.runningstate:
                    threading.Thread(target=self.arm_and_setup, args=(request, response)).start()
                    response.success = True
                    response.message = "Arm request sent correctly"
                    return response
        elif request.command_input.comtype == 't':
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
        print("drone armed and ready")
    
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
      
        
def main():
    rclpy.init()
    cflib.crtp.init_drivers()
    time.sleep(1)
    uri= uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        cf = scf.cf

        min_flight_service = MinFlightService(cf)
        
        executor = MultiThreadedExecutor()
        executor.add_node(min_flight_service)

        try:
            executor.spin()
        except KeyboardInterrupt:
            min_flight_service.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

if __name__ == '__main__':
    main()