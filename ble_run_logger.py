"""BLE host for the RunLogger IMU.

Connects to the device, sends START, streams sensor packets to a CSV file
until Ctrl+C, then sends STOP and disconnects.
"""
import asyncio
import struct
import csv
import time
from bleak import BleakClient, BleakScanner


TARGET_NAME  = "RunLogger"
DATA_UUID    = "12345678-1234-5678-1234-56789abcdef1"
CONTROL_UUID = "12345678-1234-5678-1234-56789abcdef2"

ACCEL_SCALE = 8192.0   # LSB per g
GYRO_SCALE  = 16.4     # LSB per dps

PACKET_FMT  = "<I6h"                          # uint32 + 6 x int16
PACKET_SIZE = struct.calcsize(PACKET_FMT)     # 16 bytes

OUTPUT_FILE = f"run_session_{int(time.time())}.csv"


def decode_packet(data):
    """Unpack a single 16-byte packet into (timestamp, ax, ay, az, gx, gy, gz)."""
    t_us, ax, ay, az, gx, gy, gz = struct.unpack(PACKET_FMT, data)
    return (
        t_us,
        ax / ACCEL_SCALE,
        ay / ACCEL_SCALE,
        az / ACCEL_SCALE,
        gx / GYRO_SCALE,
        gy / GYRO_SCALE,
        gz / GYRO_SCALE,
    )


async def main():
    print("Scanning for RunLogger...")
    devices = await BleakScanner.discover(timeout=8.0)

    device = next((d for d in devices if d.name and TARGET_NAME in d.name), None)
    if device is None:
        print("RunLogger not found")
        return

    print(f"Found {device.name} at {device.address}")

    sample_count = 0
    client = None

    try:
        async with BleakClient(device) as client:
            print("Connected")

            first_t_us = None
            recording = True
            last_print = time.time()

            with open(OUTPUT_FILE, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "ax", "ay", "az", "gx", "gy", "gz"])

                def on_packet(_sender, data):
                    nonlocal sample_count, first_t_us, last_print
                    if not recording or len(data) != PACKET_SIZE:
                        return

                    t_us, ax, ay, az, gx, gy, gz = decode_packet(data)

                    # convert device timestamp to seconds since first sample
                    if first_t_us is None:
                        first_t_us = t_us
                    delta_us = (t_us - first_t_us) & 0xFFFFFFFF
                    t_s = delta_us / 1e6

                    writer.writerow([f"{t_s:.6f}", ax, ay, az, gx, gy, gz])
                    sample_count += 1

                    now = time.time()
                    if now - last_print >= 2.0:
                        print(f"  {sample_count} samples, {t_s:.1f} s")
                        last_print = now

                await client.start_notify(DATA_UUID, on_packet)
                await client.write_gatt_char(CONTROL_UUID, b"START")
                print("Recording. Press Ctrl+C to stop.")

                while True:
                    await asyncio.sleep(1)

    except (asyncio.CancelledError, KeyboardInterrupt):
        pass

    finally:
        print("\nStopping...")
        try:
            if client is not None and client.is_connected:
                await client.write_gatt_char(CONTROL_UUID, b"STOP")
                await asyncio.sleep(0.3)
                await client.stop_notify(DATA_UUID)
        except Exception:
            pass
        print(f"Saved {sample_count} samples to {OUTPUT_FILE}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
