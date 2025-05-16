import rclpy
from rclpy.node import Node
from my_service.srv import ComponentError, RestartRequest
import time

class ErrorHandlerNode(Node):
    def __init__(self):
        super().__init__('error_handler')
        
        # Create the error handling service
        self.error_service = self.create_service(ComponentError, '/error_handler/handle_error', self.handle_error)
        self.get_logger().info("ErrorHandlerNode initialized and ready to handle errors.")

    def handle_error(self, request, response):
        sensor_name = request.sensor_name
        self.get_logger().info(f"Handling error for {sensor_name}")

        # Create a client to restart the sensor
        restart_client = self.create_client(RestartRequest, f'/{sensor_name}/restart')

        # Wait for the service to become available
        if not restart_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Restart service for {sensor_name} not available!")
            response.success = False
            return response  # Return the response immediately if service is not available

        # Send a restart request
        restart_request = RestartRequest.Request()
        restart_request.sensor_name = sensor_name
        future = restart_client.call_async(restart_request)
        
        # Add a callback to handle the future once it's completed
        future.add_done_callback(lambda future: self.handle_restart_result(future, response))

        # Return the response after the future completes
        # The callback will update the response and return it to the service call
        return response

    def handle_restart_result(self, future, response):
        try:
            # Process the result of the restart request
            result = future.result()
            if result.success:
                self.get_logger().info("Successfully restarted sensor.")
                response.success = True
            else:
                self.get_logger().error("Failed to restart sensor.")
                response.success = False
        except Exception as e:
            self.get_logger().error(f"Error handling restart: {e}")
            response.success = False

def main(args=None):
    rclpy.init(args=args)
    node = ErrorHandlerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
