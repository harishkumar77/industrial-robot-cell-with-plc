#!/usr/bin/env python3
"""ROS2 <-> OpenPLC bridge over Modbus TCP."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from pymodbus.client import ModbusTcpClient

STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "PICKING", 3: "WAIT_CLEAR", 9: "FAULT"}


class PlcBridge(Node):
    def __init__(self):
        super().__init__("plc_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5020)
        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        self.plc = ModbusTcpClient(host, port=port)
        if self.plc.connect():
            self.get_logger().info(f"Connected to PLC at {host}:{port}")
        else:
            self.get_logger().error(f"Can't reach PLC at {host}:{port}")

        # Machine signals. Fail-safe defaults: unsafe until told otherwise.
        self.inputs = {"door_closed": False, "estop_ok": False,
                       "part_sensor": False, "robot_done": False}
        for name in self.inputs:
            self.create_subscription(
                Bool, f"/cell/{name}",
                lambda msg, n=name: self.inputs.__setitem__(n, msg.data), 10)

        # Operator buttons as services (momentary press)
        self.pulse_until = {"start": 0.0, "stop": 0.0, "reset": 0.0}
        for name in self.pulse_until:
            self.create_service(
                Trigger, f"/cell/{name}",
                lambda req, res, n=name: self.press(n, res))

        # PLC outputs
        self.pub_conveyor = self.create_publisher(Bool, "/cell/conveyor_run", 10)
        self.pub_pick = self.create_publisher(Bool, "/cell/robot_pick", 10)
        self.pub_fault = self.create_publisher(Bool, "/cell/fault", 10)
        self.pub_state = self.create_publisher(String, "/cell/state", 10)
        self.pub_count = self.create_publisher(Int32, "/cell/part_count", 10)
        self.pub_code = self.create_publisher(Int32, "/cell/fault_code", 10)

        self.last_state = None
        self.create_timer(0.05, self.tick)   # 20 Hz

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def press(self, name, res):
        self.pulse_until[name] = self.now() + 0.3
        self.get_logger().info(f"Button: {name}")
        res.success = True
        res.message = f"{name} pressed"
        return res

    def tick(self):
        t = self.now()
        held = lambda n: t < self.pulse_until[n]
        coils = [
            held("start"),
            not held("stop"),              # Stop is normally closed
            held("reset"),
            self.inputs["estop_ok"],
            self.inputs["door_closed"],
            self.inputs["part_sensor"],
            self.inputs["robot_done"],
        ]
        try:
            self.plc.write_coils(0, coils)
            out = self.plc.read_coils(8, count=3)
            regs = self.plc.read_holding_registers(0, count=3)
        except Exception as e:
            self.get_logger().warn(f"PLC comms error: {e}", throttle_duration_sec=2.0)
            self.plc.connect()
            return
        if out.isError() or regs.isError():
            self.get_logger().warn("PLC returned an error", throttle_duration_sec=2.0)
            return

        conveyor, pick, fault = out.bits[:3]
        state, count, code = regs.registers
        name = STATE_NAMES.get(state, str(state))

        self.pub_conveyor.publish(Bool(data=conveyor))
        self.pub_pick.publish(Bool(data=pick))
        self.pub_fault.publish(Bool(data=fault))
        self.pub_state.publish(String(data=name))
        self.pub_count.publish(Int32(data=count))
        self.pub_code.publish(Int32(data=code))

        if name != self.last_state:
            self.get_logger().info(f"PLC state: {self.last_state} -> {name}")
            self.last_state = name


def main():
    rclpy.init()
    node = PlcBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plc.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
