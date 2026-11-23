import time
from pymodbus.client import ModbusTcpClient

START, STOP_NC, MOTOR = 0, 1, 2   # %QX0.0, %QX0.1, %QX0.2 -> coils 0, 1, 2

plc = ModbusTcpClient("127.0.0.1", port=5020)
print("Connected:", plc.connect())

def motor():
    return plc.read_coils(MOTOR, count=1).bits[0]

plc.write_coil(STOP_NC, True)
print("Initial motor:", motor())

plc.write_coil(START, True)
time.sleep(0.2)
plc.write_coil(START, False)
time.sleep(0.2)
print("After start pulse:", motor())

plc.write_coil(STOP_NC, False)
time.sleep(0.2)
print("After stop:", motor())
plc.write_coil(STOP_NC, True)

plc.close()
