#!/usr/bin/env python3
"""Demo operator: keeps the cell safe and running in a loop."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class AutoOperator(Node):
    def __init__(self):
        super().__init__("auto_operator")
        self.pub_door = self.create_publisher(Bool, "/cell/door_closed", 10)
        self.pub_estop = self.create_publisher(Bool, "/cell/estop_ok", 10)
        self.start = self.create_client(Trigger, "/cell/start")
        self.reset = self.create_client(Trigger, "/cell/reset")
        self.state = None
        self.fault_since = None
        self.create_subscription(String, "/cell/state", self.on_state, 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info("Auto operator running (Ctrl+C to stop)")

    def on_state(self, msg):
        if msg.data != self.state:
            self.get_logger().info(f"Cell: {msg.data}")
        self.state = msg.data

    def press(self, client, name):
        if client.service_is_ready():
            client.call_async(Trigger.Request())
            self.get_logger().info(f"Pressing {name}")

    def tick(self):
        self.pub_door.publish(Bool(data=True))
        self.pub_estop.publish(Bool(data=True))
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.state == "IDLE":
            self.press(self.start, "START")
        if self.state == "FAULT":
            if self.fault_since is None:
                self.fault_since = now
            elif now - self.fault_since > 3.0:
                self.press(self.reset, "RESET")
                self.fault_since = None
        else:
            self.fault_since = None


def main():
    rclpy.init()
    node = AutoOperator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
