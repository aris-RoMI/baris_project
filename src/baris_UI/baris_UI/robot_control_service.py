import rclpy
from rclpy.node import Node
from message.srv import RobotService  # Make sure to replace with the correct service type

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')
        

    def create_service_client(self):
        self.client = self.create_client(RobotService, 'command_client')

        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')

    def send_service_request(self, command_list):
        if self.client.service_is_ready():
            req = RobotService.Request()
            req.seq_no = command_list[0]
            req.cmd = command_list[1]
            req.par1 = command_list[2]
            req.par2 = command_list[3]
            req.par3 = command_list[4]
            req.par4 = command_list[5]
            req.par5 = command_list[6]
            future = self.client.call_async(req)
            future.add_done_callback(self.handle_service_response)  
        else:
            self.get_logger().info('Service is not ready')

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.get_logger().info(f'Failure: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()