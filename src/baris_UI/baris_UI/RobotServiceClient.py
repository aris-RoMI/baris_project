import rclpy
from rclpy.node import Node
from message.srv import RobotService
from threading import Condition

class RobotServiceClient(Node):

    def __init__(self):
        super().__init__('robot_service_client')
        self.client = self.create_client(RobotService, 'XYZ_robot/service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to be available...')
        self.request = RobotService.Request()
        self.response = None
        self.condition = Condition()

    def send_request(self, seq_no, cmd, par1=None, par2=None, par3=None, par4=None, par5=None):
        self.request.seq_no = seq_no
        self.request.cmd = cmd
        self.request.par1 = par1 if par1 else ''
        self.request.par2 = par2 if par2 else ''
        self.request.par3 = par3 if par3 else ''
        self.request.par4 = par4 if par4 else ''
        self.request.par5 = par5 if par5 else ''
        
        future = self.client.call_async(self.request)
        future.add_done_callback(self.handle_service_response)
        
        with self.condition:
            self.condition.wait()  # Wait for the future to complete
        
        return self.response
    
    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.seq_no:
                self.get_logger().info(f'Success: {response}')
            else:
                self.get_logger().info(f'Failure: {response}')
            self.response = response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.response = None
        finally:
            with self.condition:
                self.condition.notify()  # Notify the waiting thread

