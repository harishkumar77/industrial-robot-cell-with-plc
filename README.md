# PLC Controlled Robot Cell

A robot cell where a PLC is in charge. Built fully in simulation.

The PLC decides everything. When the conveyor runs, when the robot picks, when the cell stops. The simulation only does what the PLC tells it.

> Demo video: see my LinkedIn post (link coming soon)

---

## Why I built this

Most of my work has been on the robot side. ROS2, simulation, motion planning.

But in a factory, robots don't run alone. A PLC runs the cell. It controls the conveyor, reads the sensors, watches the safety door, and tells the robot when to move.

I wanted to understand that side. So I built a full cell, end to end, without any hardware.

---

## How it works

```
Isaac Sim (conveyor, photo-eye, Franka arm)
        |   ROS2 topics
        v
plc_bridge.py (ROS2 node)
        |   Modbus TCP
        v
OpenPLC running cell_logic.st  <-- the brain
```

- **OpenPLC** runs the cell logic, written in structured text.
- **plc_bridge.py** is a ROS2 node. It copies signals between ROS2 topics and the PLC over Modbus TCP. It makes no decisions.
- **isaac_cell.py** is the machine. It moves the box on the conveyor, reports the sensor, and runs the Franka pick and place when the PLC says so.
- **auto_operator.py** acts like an operator for demos. It keeps the door and e-stop marked safe, and presses Start and Reset.

The PLC is the only place where decisions are made. The rest just report or obey.

---

## The sequence

```
IDLE --start--> RUNNING --part at sensor--> PICKING --robot done--> WAIT_CLEAR
                   ^                                                    |
                   +------------ robot dropped done, part gone ---------+

Any running state --e-stop / door open / robot timeout--> FAULT --reset (only if safe)--> IDLE
```

| State | Value | What happens |
|---|---|---|
| IDLE | 0 | Waiting for start |
| RUNNING | 1 | Conveyor on |
| PICKING | 2 | Conveyor off, robot told to pick |
| WAIT_CLEAR | 3 | Waiting for the handshake to finish |
| FAULT | 9 | Everything off until reset |

**Why WAIT_CLEAR?** Both sides have to give back their signals before the next cycle. Without it, the robot's old "done" could count the next part before the robot even touches it. This two-way agreement is the handshake.

---

## Things I designed in on purpose

- **Fail-safe signals.** Stop, e-stop and the door switch are TRUE when safe. If a signal is lost, it reads FALSE and the cell stops.
- **Fail-safe startup.** The bridge treats the door and e-stop as unsafe until something confirms they are safe.
- **Latched faults.** When a fault happens, the cell stays stopped even after the cause is gone. Someone has to press reset.
- **Robot handshake with timeout.** The PLC waits for the robot to say done. If it doesn't come in time, the PLC faults instead of waiting forever.
- **Stop is not a fault.** A normal stop goes back to IDLE. No reset needed.

**Note:** this is standard PLC logic, not a certified safety system. Real cells use safety PLCs and safety relays for e-stops and doors.

---

## Fault codes

| Code | Meaning |
|---|---|
| 1 | E-stop pressed |
| 2 | Safety door opened |
| 3 | Robot timeout (no done signal in 60 s) |

---

## I/O and Modbus map

Modbus TCP on port **5020** (non-root port. Real PLCs use 502).

**Inputs to the PLC** (written by the bridge)

| Coil | Address | Signal | Meaning |
|---|---|---|---|
| 0 | %QX0.0 | Start_PB | Start button |
| 1 | %QX0.1 | Stop_NC | Stop button, TRUE = not pressed |
| 2 | %QX0.2 | Reset_PB | Reset button |
| 3 | %QX0.3 | EStop_NC | E-stop, TRUE = not pressed |
| 4 | %QX0.4 | Door_Closed | Safety door closed |
| 5 | %QX0.5 | Part_Sensor | Photo-eye sees a part |
| 6 | %QX0.6 | Robot_Done | Robot finished the pick |

**Outputs from the PLC** (read by the bridge)

| Coil | Address | Signal | Meaning |
|---|---|---|---|
| 8 | %QX1.0 | Conveyor_Run | Run the conveyor |
| 9 | %QX1.1 | Robot_Pick | Tell the robot to pick |
| 10 | %QX1.2 | Fault_Active | Cell is in fault |

**Status registers** (holding registers)

| Register | Address | Signal |
|---|---|---|
| 0 | %QW0 | State |
| 1 | %QW1 | Part_Count |
| 2 | %QW2 | Fault_Code |

`%QX1.0` is coil 8 because each byte has 8 bits. Byte 1, bit 0 is the 9th bit, which is coil 8 when counting from 0.

**Note:** in a real PLC, buttons and sensors would be physical inputs (`%IX`). I used `%QX` here because Modbus clients can write to it, and there are no real wires in the sim.

---

## ROS2 interface

| Topic / Service | Type | Direction |
|---|---|---|
| /cell/part_sensor | std_msgs/Bool | sim -> PLC |
| /cell/robot_done | std_msgs/Bool | sim -> PLC |
| /cell/door_closed | std_msgs/Bool | operator -> PLC |
| /cell/estop_ok | std_msgs/Bool | operator -> PLC |
| /cell/start, /cell/stop, /cell/reset | std_srvs/Trigger | operator -> PLC |
| /cell/conveyor_run | std_msgs/Bool | PLC -> sim |
| /cell/robot_pick | std_msgs/Bool | PLC -> sim |
| /cell/state | std_msgs/String | PLC -> all |
| /cell/part_count, /cell/fault_code | std_msgs/Int32 | PLC -> all |

Topics are used for signals that change all the time, like sensors. Services are used for one-time button presses.

---

## Repo structure

```
step1 files (root)       First ladder logic: seal-in start/stop rung
step2/cell_logic.st      The cell state machine (the PLC program)
step2/test_cell.py       Tests the PLC logic over Modbus without a sim
step3/plc_bridge.py      ROS2 <-> PLC bridge (Modbus TCP)
step3/isaac_cell.py      Isaac Sim cell: conveyor, photo-eye, Franka
step3/auto_operator.py   Demo operator that keeps the cell running
step3/run_isaac.sh       Starts Isaac Sim with its own ROS2 libraries
```

---

## How to run

**Tested on:** Ubuntu 24.04, ROS2 Jazzy, Isaac Sim 5.1, OpenPLC Runtime v3.

**1. PLC**
- Install OpenPLC Runtime v3 and open `localhost:8080`.
- Settings: enable Modbus server on port 5020.
- Programs: upload `step2/cell_logic.st` and start the PLC.

**2. Bridge** (terminal A)
```bash
source /opt/ros/jazzy/setup.bash
cd step3 && python3 plc_bridge.py
```

**3. Isaac Sim** (terminal B. Do not source system ROS2 here)
```bash
cd step3 && ./run_isaac.sh
```
Wait for "Isaac cell ready".

**4. Operator** (terminal C)
```bash
source /opt/ros/jazzy/setup.bash
cd step3 && python3 auto_operator.py
```

**Try a fault:**
```bash
ros2 topic pub --once -w 1 /cell/door_closed std_msgs/msg/Bool "{data: false}"
```
The cell stops, then the operator resets it and it runs again.

---

## Problems I hit and what I learned

- **The robot closed the gripper on air.** Isaac's pick and place controller is timed, not checked. On a longer move the timer ran out before the arm reached the box. This is exactly why the PLC waits for a real done signal instead of trusting a timer.
- **Two bridges were running at once.** Both wrote to the same PLC inputs and the cell kept faulting. Each PLC input should have only one writer.
- **The PLC showed "Running" but my new code wasn't.** The upload had failed to compile and the old program kept running. Always check the compile result.
- **Isaac Sim crashed on `import rclpy`.** Isaac Sim 5.1 uses Python 3.11 and system ROS2 Jazzy uses 3.12. The launcher script hides the system ROS2 so Isaac uses its own.
- **The box didn't move on the conveyor.** Setting velocity on a resting box did nothing because of friction. Moving it a small step each frame works.

---

## Next

- Operator screen (HMI) in Ignition
- Real wiring diagram for this cell (24 V I/O, safety relay, motor contactor)
- Heartbeat: the PLC faults if the bridge or sim stops talking
