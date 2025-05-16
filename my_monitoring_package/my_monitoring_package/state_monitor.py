import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from my_service.srv import ComponentError  # Assuming ComponentError is your error handler service type
import threading

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # Load parameters for sensors and timeout
        self.declare_parameter('sensors', ['sensor1', 'sensor2'])
        self.declare_parameter('timeout', 5.0)
        
        self.sensors = self.get_parameter('sensors').get_parameter_value().string_array_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self.sensor_states = {sensor: True for sensor in self.sensors}
        self.sensor_timers = {sensor: None for sensor in self.sensors}
        self.sensor_error_handled = {sensor: False for sensor in self.sensors}
        self.error_handler_in_progress = {sensor: False for sensor in self.sensors}  # Track error handler status

        # Subscribe to sensor state topics
        for sensor in self.sensors:
            self.create_subscription(Bool, f'/{sensor}_state', lambda msg, s=sensor: self.sensor_state_callback(msg, s), 1)

        # Error handler service client
        self.error_handler_client = self.create_client(ComponentError, '/error_handler/handle_error')
        self.wait_for_error_handler_service()

        # Periodic timer to check overall system state
        self.create_timer(1.0, self.check_system_state)

        self.get_logger().info("Monitor Node initialized and monitoring sensors.")

    def wait_for_error_handler_service(self):
        self.get_logger().info("Waiting for error handler service to become available...")
        while not self.error_handler_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Error handler service not available, retrying...")

    def sensor_state_callback(self, msg, sensor):
        self.get_logger().info(f"Received state for {sensor}: {msg.data}")

        if not msg.data:
            if not self.sensor_error_handled[sensor] and not self.error_handler_in_progress[sensor]:
                self.handle_error(sensor)
        else:
            self.sensor_states[sensor] = True
            self.sensor_error_handled[sensor] = False

            # Reset or cancel existing timer if running
            if self.sensor_timers[sensor]:
                self.sensor_timers[sensor].cancel()

            # Start a new timer to track timeout
            self.sensor_timers[sensor] = threading.Timer(self.timeout, lambda: self.trigger_timeout_error(sensor))
            self.sensor_timers[sensor].start()

    def trigger_timeout_error(self, sensor_name):
        """Triggers error handling after timeout if no new state is received."""
        if not self.sensor_error_handled[sensor_name]:
            self.get_logger().warn(f"Sensor {sensor_name} timeout detected. Triggering error handler.")
            self.handle_error(sensor_name)

    def handle_error(self, sensor_name):
        """Initiates an error-handling service call asynchronously."""
        self.get_logger().warn(f"Triggering error handler for {sensor_name}...")

        # Mark error handler as in progress
        self.error_handler_in_progress[sensor_name] = True

        request = ComponentError.Request()
        request.sensor_name = sensor_name

        future = self.error_handler_client.call_async(request)
        future.add_done_callback(lambda f: self.error_callback(f, sensor_name))

    def error_callback(self, future, sensor_name):
        """Callback to handle service call response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully handled error for {sensor_name}.")
            else:
                self.get_logger().info(f"Error handler handling {sensor_name}.")
        except Exception as e:
            self.get_logger().error(f"Service call for {sensor_name} failed: {e}")

        # Mark error as handled and error handler as complete
        self.sensor_error_handled[sensor_name] = True
        self.error_handler_in_progress[sensor_name] = False
        self.sensor_states[sensor_name] = False

    def check_system_state(self):
        """Publishes the overall system state based on sensor states."""
        overall_state = all(self.sensor_states.values())
        self.get_logger().debug(f"System state: {overall_state}")

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
