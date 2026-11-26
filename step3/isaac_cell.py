"""Isaac Sim robot cell driven by the PLC through ROS2."""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from std_msgs.msg import Bool
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, VisualCuboid
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController

# ---- Cell layout (meters) ----
BELT_X, BELT_TOP = 0.45, 0.10
PART = 0.0515
SPAWN_Y, SENSOR_Y = -0.55, 0.0
BELT_SPEED = 0.12
N_PARTS = 4
PARK = [np.array([-0.6, -0.3 + 0.15 * i, PART / 2]) for i in range(N_PARTS)]
PLACE = [np.array([-0.10 + 0.066 * i, 0.5, PART / 2 + 0.01]) for i in range(N_PARTS)]

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.scene.add(FixedCuboid(prim_path="/World/Conveyor", name="conveyor",
                            position=np.array([BELT_X, 0.0, BELT_TOP / 2]),
                            scale=np.array([0.2, 1.3, BELT_TOP]),
                            color=np.array([0.2, 0.2, 0.2])))
world.scene.add(VisualCuboid(prim_path="/World/PhotoEye", name="photo_eye",
                             position=np.array([BELT_X, SENSOR_Y, BELT_TOP + PART / 2]),
                             scale=np.array([0.3, 0.005, 0.005]),
                             color=np.array([1.0, 0.0, 0.0])))
world.scene.add(VisualCuboid(prim_path="/World/Bin", name="bin",
                             position=np.array([0.0, 0.5, 0.005]),
                             scale=np.array([0.32, 0.12, 0.01]),
                             color=np.array([0.2, 0.4, 0.8])))
parts = [world.scene.add(DynamicCuboid(prim_path=f"/World/Part{i}", name=f"part{i}",
                                       position=PARK[i], scale=np.array([PART] * 3),
                                       color=np.array([0.95, 0.55, 0.1]), mass=0.05))
         for i in range(N_PARTS)]
franka = world.scene.add(Franka(prim_path="/World/Franka", name="franka"))
controller = PickPlaceController(name="pick_place", gripper=franka.gripper,
                                 robot_articulation=franka,
                                 events_dt=[0.008, 0.008, 0.1, 0.2, 0.008, 0.004, 0.008, 1.0, 0.02, 0.2])
from pxr import UsdLux
UsdLux.DomeLight.Define(world.stage, "/World/DomeLight").CreateIntensityAttr(1000.0)
world.reset()
franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

# ---- ROS2 ----
rclpy.init()
node = rclpy.create_node("isaac_cell")
pub_sensor = node.create_publisher(Bool, "/cell/part_sensor", 10)
pub_done = node.create_publisher(Bool, "/cell/robot_done", 10)
cmd = {"conveyor_run": False, "robot_pick": False, "jam": False}
for topic, key in [("/cell/conveyor_run", "conveyor_run"),
                   ("/cell/robot_pick", "robot_pick"),
                   ("/sim/jam_robot", "jam")]:
    node.create_subscription(Bool, topic, lambda m, k=key: cmd.__setitem__(k, m.data), 10)
node.get_logger().info("Isaac cell ready")

current, robot_done, picking = 0, False, False
spawn_needed, reset_needed = True, False


def spawn(part):
    part.set_world_pose(position=np.array([BELT_X, SPAWN_Y, BELT_TOP + PART / 2]),
                        orientation=np.array([1.0, 0.0, 0.0, 0.0]))
    part.set_linear_velocity(np.zeros(3))


while simulation_app.is_running():
    world.step(render=True)
    rclpy.spin_once(node, timeout_sec=0.0)

    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if not world.is_playing():
        continue
    if reset_needed:
        world.reset()
        controller.reset()
        franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
        current, robot_done, picking, spawn_needed, reset_needed = 0, False, False, True, False

    part = parts[current]
    if spawn_needed:
        spawn(part)
        spawn_needed = False

    pos, _ = part.get_world_pose()
    on_belt = (abs(pos[0] - BELT_X) < 0.12 and pos[2] < BELT_TOP + 0.06
               and -0.66 < pos[1] < 0.66)
    part_sensor = on_belt and abs(pos[1] - SENSOR_Y) < 0.03

    # Conveyor: only moves the part when the PLC says so
    if on_belt and not picking:
        if cmd["conveyor_run"]:
            pos = pos + np.array([0.0, BELT_SPEED * world.get_physics_dt(), 0.0])
            part.set_world_pose(position=pos)
        part.set_linear_velocity(np.zeros(3))

    # Robot: picks only when the PLC says so
    if cmd["robot_pick"] and not robot_done and not cmd["jam"]:
        if not picking:
            pick_target = pos.copy()
        picking = True
        actions = controller.forward(picking_position=pick_target,
                                     placing_position=PLACE[current],
                                     current_joint_positions=franka.get_joint_positions(),
                                     end_effector_offset=np.array([0, 0.005, 0]))
        franka.apply_action(actions)
        if controller.is_done():
            robot_done, picking = True, False
            node.get_logger().info(f"Placed part {current}")

    # Handshake: PLC dropped the pick command
    if not cmd["robot_pick"]:
        if robot_done:
            robot_done = False
            controller.reset()
            current = (current + 1) % N_PARTS
            spawn_needed = True
        elif picking:            # aborted mid-pick (fault)
            picking = False
            controller.reset()

    # Part fell off somewhere: put it back on the belt
    lost = (not on_belt and not picking and not robot_done and cmd["conveyor_run"]
            and np.linalg.norm(pos[:2] - PLACE[current][:2]) > 0.05)
    if lost:
        spawn_needed = True

    pub_sensor.publish(Bool(data=bool(part_sensor)))
    pub_done.publish(Bool(data=bool(robot_done)))

node.destroy_node()
rclpy.shutdown()
simulation_app.close()
