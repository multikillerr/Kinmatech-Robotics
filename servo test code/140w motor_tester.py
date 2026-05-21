#!/usr/bin/env python3
import time
import csv
import serial
import minimalmodbus
from datetime import datetime

# ---------------- USER SETTINGS ----------------
PORT  = "/dev/tty.usbserial-1110"
SLAVE = 1
BAUD  = 115200

POLL_HZ = 40.0
DT = 1.0 / POLL_HZ

RPM_MAX = 2400  # hard ceiling

ACCEL = 200
DECEL = 200

# software ramp (adds safety + repeatability)
RAMP_STEP_RPM = 200
RAMP_STEP_DT  = 0.06

# direction change safety
ZERO_DWELL_S   = 0.25   # dwell at 0 before reversing
REV_ENTRY_RPM  = 200    # small entry speed after reversal

TOTAL_SECONDS = 10 * 60  # 10 minutes

LOG_CSV = f"jkong_dir_burst_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
# ------------------------------------------------

# Registers
REG_CLEAR_ERROR_FLAG = 8200
REG_ENABLE_CMD       = 8961
REG_RUNSTATE         = 8201
REG_MODE             = 24672
REG_CONTROLWORD      = 24640
REG_ERROR_CODE       = 24639

REG_BUS_VOLT_U16     = 8195
REG_ACTUAL_POS_I32   = 24676
REG_ACTUAL_SPD_I32   = 24684

REG_ACCEL_U32        = 24707
REG_DECEL_U32        = 24708
REG_SPEED_TARGET_I32 = 24831

# Controlwords / modes
CW_CLEAR_ERR   = 128
CW_CUT_POWER   = 6
CW_EN_SUPPLY   = 7
CW_RUN_ENABLE  = 15
CW_TRIGGER_RUN = 31
CW_STOP        = 271
MODE_SPEED     = 3

def clamp_rpm(x:int)->int:
    if x > RPM_MAX:  return RPM_MAX
    if x < -RPM_MAX: return -RPM_MAX
    return int(x)

def make_instrument():
    ins = minimalmodbus.Instrument(PORT, SLAVE)
    ins.mode = minimalmodbus.MODE_RTU
    ins.serial.baudrate = BAUD
    ins.serial.bytesize = 8
    ins.serial.parity   = serial.PARITY_NONE
    ins.serial.stopbits = 1
    ins.serial.timeout  = 0.30
    ins.clear_buffers_before_each_transaction = True
    return ins

ins = make_instrument()

def wr_u16(reg, val):
    return ins.write_register(reg, val, 0, 6, False)

def rd_u16(reg):
    return ins.read_register(reg, 0, 3, False)

def rd_i32(reg):
    return ins.read_long(reg, 3, True)

def wr_u32(reg, val_u32):
    hi = (val_u32 >> 16) & 0xFFFF
    lo = (val_u32 >>  0) & 0xFFFF
    return ins.write_registers(reg, [hi, lo])  # FC16

def wr_i32(reg, val_i32):
    val_i32 = clamp_rpm(val_i32)
    if val_i32 < 0:
        val_i32 = (1 << 32) + val_i32
    return wr_u32(reg, val_i32)

def clear_alarms():
    wr_u16(REG_CLEAR_ERROR_FLAG, 1)
    time.sleep(0.03)
    wr_u16(REG_CONTROLWORD, CW_CLEAR_ERR)
    time.sleep(0.08)

def enable_drive():
    wr_u16(REG_ENABLE_CMD, 1)
    time.sleep(0.03)
    wr_u16(REG_CONTROLWORD, CW_CUT_POWER)
    time.sleep(0.03)
    wr_u16(REG_CONTROLWORD, CW_EN_SUPPLY)
    time.sleep(0.03)
    wr_u16(REG_CONTROLWORD, CW_RUN_ENABLE)
    time.sleep(0.08)

def start_speed_mode():
    wr_u16(REG_MODE, MODE_SPEED)
    time.sleep(0.03)
    wr_u32(REG_ACCEL_U32, ACCEL)
    wr_u32(REG_DECEL_U32, DECEL)
    time.sleep(0.03)
    trigger_run()

def trigger_run():
    # Re-arm and edge-trigger. Safe to call often.
    wr_u16(REG_CONTROLWORD, CW_RUN_ENABLE)
    time.sleep(0.01)
    wr_u16(REG_CONTROLWORD, CW_TRIGGER_RUN)
    time.sleep(0.02)

def set_speed(rpm):
    wr_i32(REG_SPEED_TARGET_I32, rpm)
    trigger_run()

def soft_stop():
    set_speed(0)
    time.sleep(0.20)

def hard_stop():
    try:
        wr_u16(REG_CONTROLWORD, CW_STOP)
    except Exception:
        pass
    time.sleep(0.25)

def read_snapshot():
    err = rd_u16(REG_ERROR_CODE)
    rs  = rd_u16(REG_RUNSTATE)
    pos = rd_i32(REG_ACTUAL_POS_I32)
    spd = rd_i32(REG_ACTUAL_SPD_I32)
    bv  = rd_u16(REG_BUS_VOLT_U16)
    return err, rs, pos, spd, bv

def recover(tag, writer):
    # We only treat non-zero err as a true fault.
    try:
        err, rs, pos, spd, bv = read_snapshot()
        writer.writerow([time.time(), tag, "RECOVER_BEGIN", pos, spd, bv, rs, f"err=0x{err:04X}"])
    except Exception as e:
        writer.writerow([time.time(), tag, "RECOVER_READ_FAIL", "", "", "", "", str(e)])
        return False

    try:
        # gentle settle
        wr_i32(REG_SPEED_TARGET_I32, 0)
        time.sleep(0.10)
    except Exception:
        pass

    try:
        clear_alarms()
        enable_drive()
        start_speed_mode()
        # force arm/trigger again even if err==0
        trigger_run()
        time.sleep(0.10)

        err2, rs2, pos2, spd2, bv2 = read_snapshot()
        writer.writerow([time.time(), tag, "RECOVER_END", pos2, spd2, bv2, rs2, f"err=0x{err2:04X}"])
        return (err2 == 0)  # runstate can be 0 at speed=0; that's fine
    except Exception as e:
        writer.writerow([time.time(), tag, "RECOVER_FAIL", "", "", "", "", str(e)])
        return False

def ramp_to(target_rpm, writer, tag):
    target_rpm = clamp_rpm(target_rpm)
    try:
        _, _, _, spd, _ = read_snapshot()
        cmd = int(spd)
    except Exception:
        cmd = 0

    step = RAMP_STEP_RPM if target_rpm >= cmd else -RAMP_STEP_RPM

    while (step > 0 and cmd < target_rpm) or (step < 0 and cmd > target_rpm):
        cmd += step
        if (step > 0 and cmd > target_rpm) or (step < 0 and cmd < target_rpm):
            cmd = target_rpm

        set_speed(cmd)

        err, rs, pos, spd2, bv = read_snapshot()
        writer.writerow([time.time(), tag, "RAMP", pos, spd2, bv, rs, f"cmd={cmd} err=0x{err:04X}"])

        if err != 0:
            if not recover(tag, writer):
                return False

        time.sleep(RAMP_STEP_DT)

    return True

def hold(tag, seconds, writer, note="", ignore_rs=True):
    t0 = time.perf_counter()
    while True:
        t = time.perf_counter() - t0
        if t >= seconds:
            return True

        err, rs, pos, spd, bv = read_snapshot()
        writer.writerow([time.time(), tag, "HOLD", pos, spd, bv, rs, f"{note} err=0x{err:04X}"])

        # Only err != 0 is treated as a fault.
        if err != 0:
            if not recover(tag, writer):
                return False

        time.sleep(DT)

def safe_reverse_to(target_rpm, writer, tag):
    target_rpm = clamp_rpm(target_rpm)

    # 1) ramp to zero
    if not ramp_to(0, writer, tag + "_TO_ZERO"):
        return False

    # 2) dwell at zero (runstate may drop to 0; that's fine)
    if not hold(tag + "_ZERO_DWELL", ZERO_DWELL_S, writer, "dwell@0"):
        return False

    # 3) entry speed in new direction
    entry = REV_ENTRY_RPM if target_rpm >= 0 else -REV_ENTRY_RPM
    if entry != 0:
        if not ramp_to(entry, writer, tag + "_ENTRY"):
            return False
        if not hold(tag + "_ENTRY_HOLD", 0.15, writer, f"entry={entry}"):
            return False

    # 4) ramp to target
    return ramp_to(target_rpm, writer, tag + "_UP")

def main():
    with open(LOG_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["unix_s","segment","phase","pos","spd","busV","runstate","note"])

        clear_alarms()
        enable_drive()
        start_speed_mode()
        w.writerow([time.time(), "INIT", "READY", "", "", "", "", ""])

        start = time.perf_counter()
        cycle = 0

        while (time.perf_counter() - start) < TOTAL_SECONDS:
            cycle += 1

            # A: forward burst to 2400 then drop
            if not ramp_to(+2400, w, f"C{cycle}_A_FWD_BURST"):
                break
            if not hold(f"C{cycle}_A_FWD_BURST", 0.35, w, "hold@2400"):
                break
            if not ramp_to(+600, w, f"C{cycle}_A_FWD_DROP"):
                break
            if not hold(f"C{cycle}_A_FWD_DROP", 0.30, w, "hold@600"):
                break

            # B: reverse burst (safe reverse via zero)
            if not safe_reverse_to(-2400, w, f"C{cycle}_B_REV_BURST"):
                break
            if not hold(f"C{cycle}_B_REV_BURST_HOLD", 0.35, w, "hold@-2400"):
                break
            if not ramp_to(-600, w, f"C{cycle}_B_REV_DROP"):
                break
            if not hold(f"C{cycle}_B_REV_DROP", 0.30, w, "hold@-600"):
                break

            # C: longer holds alternating direction (via zero)
            if not ramp_to(-1500, w, f"C{cycle}_C_REV_HOLD"):
                break
            if not hold(f"C{cycle}_C_REV_HOLD", 2.0, w, "hold@-1500"):
                break

            if not safe_reverse_to(+1500, w, f"C{cycle}_C_FWD_HOLD"):
                break
            if not hold(f"C{cycle}_C_FWD_HOLD", 2.0, w, "hold@+1500"):
                break

            # D: regen-heavy controlled stop
            if not ramp_to(+2200, w, f"C{cycle}_D_REGEN_UP"):
                break
            if not hold(f"C{cycle}_D_REGEN_UP", 0.60, w, "hold@2200"):
                break
            if not ramp_to(0, w, f"C{cycle}_D_REGEN_DOWN"):
                break
            if not hold(f"C{cycle}_D_REGEN_SETTLE", 0.60, w, "settle@0"):
                break

        # final stop
        try:
            soft_stop()
            hard_stop()
        except Exception:
            pass

        w.writerow([time.time(), "DONE", "STOPPED", "", "", "", "", f"Saved {LOG_CSV}"])

    print(f"Done. Log saved to: {LOG_CSV}")

if __name__ == "__main__":
    main()