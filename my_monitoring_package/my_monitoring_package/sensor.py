import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from my_service.srv import RestartRequest
import random
import time

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.sensor_publishers = {}
        self.restart_services = {}
        self.declare_parameter('sensor_names', ['sensor1', 'sensor2'])

        self.sensor_names = self.get_parameter('sensor_names').value
        self.sensor_states = {sensor_name: True for sensor_name in self.sensor_names}  # Track sensor states

        # Create publishers and restart services for each sensor
        for sensor_name in self.sensor_names:
            self.sensor_publishers[sensor_name] = self.create_publisher(Bool, f"/{sensor_name}_state", 2)
            self.restart_services[sensor_name] = self.create_service(RestartRequest, f"/{sensor_name}/restart", self.handle_restart)
            self.get_logger().info(f"Restart service for {sensor_name} available.")

        # Timer to periodically check and publish sensor states
        self.timer = self.create_timer(1.0, self.publish_states)

    def publish_states(self):
        """
        Periodically publishes sensor state.
        """
        for sensor_name in self.sensor_names:
            # Randomly decide the state with 20% False and 80% True
            state = random.choices([True, False], weights=[0.8, 0.2])[0]

            # Publish the state
            self.sensor_publishers[sensor_name].publish(Bool(data=state))
            # Update the internal state tracker
            self.sensor_states[sensor_name] = state
            self.get_logger().info(f"{sensor_name} state: {state}")

    def handle_restart(self, request, response):
        """
        Handle the restart service for a sensor.
        """
        sensor_name = request.sensor_name
        self.get_logger().info(f"Restarting {sensor_name}...")

        # Simulate a restart operation with a small delay (you can replace this with actual logic)
        time.sleep(2)  # Simulate restart duration

        # Here we simulate a successful restart (remove failure logic if not needed)
        self.get_logger().info(f"{sensor_name} restarted successfully.")
        
        # Return the success status of the restart attempt
        response.success = True  # Always return success since restart is guaranteed

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
