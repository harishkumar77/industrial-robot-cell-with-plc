PROGRAM cell_logic
 VAR
 Start_PB AT %QX0.0 : BOOL;
 Stop_NC AT %QX0.1 : BOOL := TRUE;
 Reset_PB AT %QX0.2 : BOOL;
 EStop_NC AT %QX0.3 : BOOL := TRUE;
 Door_Closed AT %QX0.4 : BOOL := TRUE;
 Part_Sensor AT %QX0.5 : BOOL;
 Robot_Done AT %QX0.6 : BOOL;
 Conveyor_Run AT %QX1.0 : BOOL;
 Robot_Pick AT %QX1.1 : BOOL;
 Fault_Active AT %QX1.2 : BOOL;
 State AT %QW0 : INT := 0;
 Part_Count AT %QW1 : INT := 0;
 Fault_Code AT %QW2 : INT := 0;
 END_VAR
 VAR
 Robot_Timer : TON;
 Safe : BOOL;
 END_VAR

 Safe := EStop_NC AND Door_Closed;

 (* Safety checks first, every scan *)
 IF State <> 0 AND State <> 9 THEN
 IF NOT EStop_NC THEN
 Fault_Code := 1; State := 9;
 ELSIF NOT Door_Closed THEN
 Fault_Code := 2; State := 9;
 ELSIF NOT Stop_NC THEN
 State := 0;
 END_IF;
 END_IF;

 (* Robot timeout: counts only while waiting for the robot *)
 Robot_Timer(IN := (State = 2), PT := T#10s);

 CASE State OF
 0: (* IDLE *)
 Conveyor_Run := FALSE;
 Robot_Pick := FALSE;
 IF Start_PB AND Stop_NC AND Safe THEN
 State := 1;
 END_IF;

 1: (* RUNNING *)
 Conveyor_Run := TRUE;
 Robot_Pick := FALSE;
 IF Part_Sensor THEN
 State := 2;
 END_IF;

 2: (* PICKING *)
 Conveyor_Run := FALSE;
 Robot_Pick := TRUE;
 IF Robot_Done THEN
 Part_Count := Part_Count + 1;
 State := 3;
 ELSIF Robot_Timer.Q THEN
 Fault_Code := 3;
 State := 9;
 END_IF;

 3: (* WAIT_CLEAR: robot drops Done, part is gone *)
 Conveyor_Run := FALSE;
 Robot_Pick := FALSE;
 IF NOT Robot_Done AND NOT Part_Sensor THEN
 State := 1;
 END_IF;

 9: (* FAULT: latched until Reset *)
 Conveyor_Run := FALSE;
 Robot_Pick := FALSE;
 IF Reset_PB AND Safe THEN
 Fault_Code := 0;
 State := 0;
 END_IF;
 END_CASE;

 Fault_Active := (State = 9);
END_PROGRAM

CONFIGURATION Config0
 RESOURCE Res0 ON PLC
 TASK task0(INTERVAL := T#20ms, PRIORITY := 0);
 PROGRAM instance0 WITH task0 : cell_logic;
 END_RESOURCE
END_CONFIGURATION
