import rclpy
from rclpy.node import Node
from message.srv import RobotService
from library.Constants import RobotCommand, RobotParameter

class RobotServiceClient(Node):

    def __init__(self):
        super().__init__('robot_service_client')
        self.client = self.create_client(RobotService, 'XYZ_robot/service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to be available...')
        self.request = RobotService.Request()

    def send_request(self, seq_no, cmd, par1=None, par2=None, par3=None, par4=None, par5=None):
        self.request.seq_no = seq_no
        self.request.cmd = cmd
        self.request.par1 = par1 if par1 else ''
        self.request.par2 = par2 if par2 else ''
        self.request.par3 = par3 if par3 else ''
        self.request.par4 = par4 if par4 else ''
        self.request.par5 = par5 if par5 else ''
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = RobotServiceClient()

    seq_no = '1'  # 예시 seq_no
    cmd = RobotCommand.HOME_NORMAL  # 실행할 명령
    par1 = RobotParameter.ZERO

    response = client.send_request(seq_no, cmd, par1)
    print(f"응답: {response}")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
