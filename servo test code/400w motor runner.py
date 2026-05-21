#!/usr/bin/env python3
import time
from pymodbus.client import ModbusSerialClient

PORT = "/dev/tty.usbmodem594C0251071"
BAUD = 115200
SID  = 1

REG_MODE_SELECT = 0x2109
REG_OP_MODE     = 0x2310
REG_POS_MODE    = 0x2311
REG_TARGET_POS  = 0x2320
REG_TARGET_SPD  = 0x2321
REG_ACC         = 0x2322
REG_DEC         = 0x2323
REG_TRIGGER     = 0x2316
REG_ENABLE      = 0x2301
REG_FAULT       = 0x603F
REG_FAULT_RST   = 0x2008

def i32_to_regs(val):
    if val < 0:
        val = (1 << 32) + val
    return [(val >> 16) & 0xFFFF, val & 0xFFFF]

def write_u16(c, addr, val):
    c.write_register(addr, int(val) & 0xFFFF, slave=SID)

def write_i32(c, addr, val):
    c.write_registers(addr, i32_to_regs(int(val)), slave=SID)

def setup_drive(c):

    fault = c.read_holding_registers(REG_FAULT, 1, slave=SID)
    if not fault.isError() and fault.registers[0] != 0:
        write_u16(c, REG_FAULT_RST, 1)
        time.sleep(0.2)

    write_u16(c, REG_ENABLE, 1)
    time.sleep(0.1)

    write_u16(c, REG_MODE_SELECT, 1)
    write_u16(c, REG_OP_MODE, 3)   # KEEP MODE 3
    write_u16(c, REG_POS_MODE, 1)  # Absolute (as your working code)

def trigger(c):
    write_u16(c, REG_TRIGGER, 0)
    time.sleep(0.005)
    write_u16(c, REG_TRIGGER, 1)

def main():

    client = ModbusSerialClient(port=PORT, baudrate=BAUD, timeout=0.2)
    if not client.connect():
        print("Connection failed")
        return

    setup_drive(client)

    seg_count = int(input("Number of segments: "))

    for i in range(seg_count):
        print(f"\n--- Segment {i+1} ---")
        pos  = int(input("Target position: "))
        speed = int(input("Speed (rpm): "))
        acc   = int(input("Acceleration: "))
        dec   = int(input("Deceleration: "))

        write_i32(client, REG_TARGET_POS, pos)
        write_u16(client, REG_TARGET_SPD, speed)
        write_u16(client, REG_ACC, acc)
        write_u16(client, REG_DEC, dec)

        trigger(client)

        time.sleep(0.02)   # tune for blending

    print("Segments sent.")

    client.close()

if __name__ == "__main__":
    main()