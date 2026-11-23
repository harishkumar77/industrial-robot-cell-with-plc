import math
import time
import pybullet as p
import pybullet_data
from pymodbus.client import ModbusTcpClient

# ---- PLC connection (must match cell_logic.st) ----
PLC_HOST, PLC_PORT = "127.0.0.1", 5020
# Coils 0-6 (we write): Start, Stop_NC, Reset, EStop_NC, Door_Closed, Part_Sensor, Robot_Done
# Coils 8-10 (we read): Conveyor_Run, Robot_Pick, Fault_Active
# Holding regs 0-2 (we read): State, Part_Count, Fault_Code
STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "PICKING", 3: "WAIT_CLEAR", 9: "FAULT"}
FAULT_TEXT = {0: "", 1: "E-stop pressed", 2: "Door open", 3: "Robot timeout"}

# ---- Cell geometry ----
DT = 1 / 60
BELT_SPEED = 0.15
BELT_Z = 0.40
PART_HALF = 0.04
SPAWN = [-0.9, 0.0, BELT_Z + PART_HALF]
BELT_END_X = 0.55
SENSOR_X = 0.30
BIN_POS = [0.75, 0.85, 0.05]
HOME = [0.30, 0.35, 0.80]

plc = ModbusTcpClient(PLC_HOST, port=PLC_PORT)
if not plc.connect():
    raise SystemExit("Can't reach the PLC. Is OpenPLC running with Modbus on port 5020?")

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.resetDebugVisualizerCamera(1.9, 50, -30, [0.1, 0.3, 0.3])
p.loadURDF("plane.urdf")


def box(half, pos, color):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    return p.createMultiBody(0, col, vis, pos)


box([0.8, 0.15, BELT_Z / 2], [-0.2, 0, BELT_Z / 2], [0.25, 0.25, 0.25, 1])   # conveyor
box([0.12, 0.12, 0.05], BIN_POS, [0.2, 0.4, 0.8, 1])                         # drop bin
door = box([0.35, 0.01, 0.35], [0.3, -0.45, 0.35], [0.2, 0.8, 0.2, 0.5])    # safety door
robot = p.loadURDF("kuka_iiwa/model.urdf", [0.3, 0.55, 0], useFixedBase=True)
EE = 6
DOWN = p.getQuaternionFromEuler([0, math.pi, 0])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# ---- Operator panel (PyBullet side panel buttons) ----
btn = {name: p.addUserDebugParameter(name, 1, 0, 0) for name in
       ["Start", "Stop", "Reset", "E-stop (toggle)", "Door open/close", "Jam robot (toggle)"]}
last_clicks = {name: 0 for name in btn}
pulse_until = {"Start": 0.0, "Stop": 0.0, "Reset": 0.0}


def clicks(name):
    return int(p.readUserDebugParameter(btn[name]))


# ---- Robot: simple waypoint follower using IK ----
class Robot:
    def __init__(self):
        self.pos = list(HOME)
        self.queue = []
        self.seg = None
        self.holding = False
        self.done = False
        self.move(HOME)

    def move(self, target):
        joints = p.calculateInverseKinematics(robot, EE, target, DOWN, maxNumIterations=100)
        for j in range(7):
            p.resetJointState(robot, j, joints[j])
        self.pos = list(target)

    def busy(self):
        return self.seg is not None or bool(self.queue)

    def update(self):
        if self.seg is None:
            if not self.queue:
                return None
            target, secs, event = self.queue.pop(0)
            self.seg = [list(self.pos), target, secs, 0.0, event]
        start, target, secs, t, event = self.seg
        t = min(t + DT, secs)
        self.seg[3] = t
        a = t / secs
        self.move([s + (e - s) * a for s, e in zip(start, target)])
        if t >= secs:
            self.seg = None
            return event
        return None


arm = Robot()
part, part_pos = None, None
spawn_timer = 0.0
beam = p.addUserDebugLine([0, 0, 0], [0, 0, 0.01], [0, 1, 0], 2)
status = p.addUserDebugText("", [-0.7, 0, 1.1], [0, 0, 0], 1.4)
last_status = None

print("Sim running. Use the buttons in the right-hand panel. Ctrl+C to quit.")
try:
    while True:
        now = time.time()

        # Operator buttons
        for name in pulse_until:
            c = clicks(name)
            if c != last_clicks[name]:
                last_clicks[name] = c
                pulse_until[name] = now + 0.3
        estop_pressed = clicks("E-stop (toggle)") % 2 == 1
        door_open = clicks("Door open/close") % 2 == 1
        jammed = clicks("Jam robot (toggle)") % 2 == 1

        # Photo-eye sensor (ray across the belt)
        a, b = [SENSOR_X, -0.3, SPAWN[2]], [SENSOR_X, 0.3, SPAWN[2]]
        hit = p.rayTest(a, b)[0][0]
        part_sensor = part is not None and hit == part
        beam = p.addUserDebugLine(a, b, [1, 0, 0] if part_sensor else [0, 1, 0], 3,
                                  replaceItemUniqueId=beam)

        # Write inputs to the PLC
        plc.write_coils(0, [
            now < pulse_until["Start"],
            not (now < pulse_until["Stop"]),     # Stop is normally closed
            now < pulse_until["Reset"],
            not estop_pressed,                   # E-stop is normally closed
            not door_open,
            part_sensor,
            arm.done,
        ])

        # Read PLC outputs
        conveyor_run, robot_pick, fault = plc.read_coils(8, count=3).bits[:3]
        state, count, code = plc.read_holding_registers(0, count=3).registers

        # Conveyor
        if part is None:
            spawn_timer -= DT
            if spawn_timer <= 0:
                part_pos = list(SPAWN)
                part = box([PART_HALF] * 3, part_pos, [0.95, 0.55, 0.1, 1])
        elif conveyor_run and not arm.holding:
            part_pos[0] = min(part_pos[0] + BELT_SPEED * DT, BELT_END_X)

        # Robot handshake
        if not robot_pick:
            arm.done = False
        if robot_pick and not arm.busy() and not arm.done and not jammed and part is not None:
            x, y, z = part_pos
            arm.queue = [
                ([x, y, z + 0.25], 1.0, None),
                ([x, y, z + 0.07], 0.7, "grab"),
                ([x, y, z + 0.25], 0.7, None),
                ([BIN_POS[0], BIN_POS[1], 0.35], 1.2, "release"),
                (HOME, 1.0, None),
            ]
        event = arm.update()
        if event == "grab":
            arm.holding = True
        elif event == "release":
            arm.holding = False
            p.removeBody(part)
            part, part_pos = None, None
            arm.done = True
            spawn_timer = 1.0
        if arm.holding and part is not None:
            ee = p.getLinkState(robot, EE)[0]
            part_pos = [ee[0], ee[1], ee[2] - 0.07]
        if part is not None:
            p.resetBasePositionAndOrientation(part, part_pos, [0, 0, 0, 1])

        # Visuals: door color and status text
        p.changeVisualShape(door, -1, rgbaColor=[0.9, 0.2, 0.2, 0.2] if door_open else [0.2, 0.8, 0.2, 0.5])
        text = f"{STATE_NAMES.get(state, state)}  parts={count}"
        if code:
            text += f"  FAULT: {FAULT_TEXT.get(code, code)}"
        if estop_pressed:
            text += "  [E-STOP]"
        if text != last_status:
            color = [0.8, 0, 0] if fault else ([0, 0.6, 0] if state else [0.3, 0.3, 0.3])
            status = p.addUserDebugText(text, [-0.7, 0, 1.1], color, 1.4, replaceItemUniqueId=status)
            last_status = text

        time.sleep(max(0, DT - (time.time() - now)))
except KeyboardInterrupt:
    pass
finally:
    plc.close()
    p.disconnect()
