import rclpy
from rclpy.node import Node
from message.srv import DispenseService
from threading import Condition

class DispenseServiceClient(Node):

    def __init__(self):
        super().__init__('dispense_service_client')
        self.client = self.create_client(DispenseService, 'XYZ_dispenser/service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for dispense service to be available...')
        self.request = DispenseService.Request()
        self.response = None
        self.condition = Condition()

    def send_request(self, seq_no, dev_id, command):
        self.request.seq_no = seq_no
        self.request.dev_id = dev_id
        self.request.command = command if command else ''

        future = self.client.call_async(self.request)
        future.add_done_callback(self.handle_service_response)
        
        with self.condition:
            self.condition.wait()  # Wait for the future to complete
        
        return self.response
    
    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.seq_no:
                self.get_logger().info(f'Dispense Success: {response}')
            else:
                self.get_logger().info(f'Dispense Failure: {response}')
            self.response = response
        except Exception as e:
            self.get_logger().error(f'Dispense Service call failed: {e}')
            self.response = None
        finally:
            with self.condition:
                self.condition.notify()  # Notify the waiting thread