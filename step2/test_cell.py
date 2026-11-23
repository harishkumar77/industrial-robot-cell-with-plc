import time
from pymodbus.client import ModbusTcpClient

plc = ModbusTcpClient("127.0.0.1", port=5020)
plc.connect()

START, STOP, RESET, ESTOP, DOOR, PART, DONE = range(7)   # %QX0.0 - %QX0.6
NAMES = {0: "IDLE", 1: "RUNNING", 2: "PICKING", 3: "WAIT_CLEAR", 9: "FAULT"}

def w(addr, val):
    plc.write_coil(addr, val)
    time.sleep(0.15)

def pulse(addr):
    w(addr, True)
    w(addr, False)

def show(step):
    c = plc.read_coils(8, count=3).bits              # %QX1.0 - %QX1.2
    r = plc.read_holding_registers(0, count=3).registers   # %QW0 - %QW2
    print(f"{step:<30} state={NAMES.get(r[0], r[0]):<10} "
          f"conveyor={c[0]!s:<5} pick={c[1]!s:<5} fault={c[2]!s:<5} "
          f"parts={r[1]} code={r[2]}")

# Healthy machine, nothing happening
for a, v in [(STOP, True), (ESTOP, True), (DOOR, True), (PART, False), (DONE, False)]:
    w(a, v)
show("Idle")

pulse(START);              show("Pressed start")
w(PART, True);             show("Part arrives")
w(DONE, True);             show("Robot says done")
w(PART, False); w(DONE, False); show("Handshake cleared")

w(PART, True);             show("Second part arrives")
print("...robot never answers, waiting 11 s")
time.sleep(11);            show("Robot timeout")
w(PART, False); pulse(RESET); show("Reset")

pulse(START);              show("Start again")
w(DOOR, False);            show("Door opened while running")
w(DOOR, True); pulse(RESET); show("Door closed + reset")

plc.close()
