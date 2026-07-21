#!/usr/bin/env python3
"""Minimal six-motor SocketCAN responder for Pi Arm integration tests."""

from __future__ import annotations

import argparse
import signal
import socket
import struct
from dataclasses import dataclass

CAN_FRAME = struct.Struct("=IB3x8s")
CAN_BASE = 0x140


@dataclass
class Motor:
    angle_raw: int = 0
    speed_dps: int = 0
    enabled: bool = False
    error: int = 0


def signed_56_le(value: int) -> bytes:
    return int(value).to_bytes(7, byteorder="little", signed=True)


def response(motor: Motor, request: bytes) -> bytes | None:
    command = request[0]
    if command == 0x92:
        return bytes([command]) + signed_56_le(motor.angle_raw)
    if command in (0x9C, 0xA4):
        if command == 0xA4:
            motor.angle_raw = int.from_bytes(request[4:8], "little", signed=True)
        return struct.pack("<Bb h h H", command, 25, 0, motor.speed_dps, 0)
    if command == 0x9A:
        motor_state = 0x30 if motor.enabled else 0x01
        return struct.pack("<Bb h h BB", command, 25, 2400, 0, motor_state, motor.error)
    if command == 0x88:
        motor.enabled = True
    elif command == 0x80:
        motor.enabled = False
    elif command == 0x9B:
        motor.error = 0
    elif command == 0x19:
        motor.angle_raw = 0
    else:
        return None
    return bytes([command]) + bytes(7)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", nargs="?", default="vcan0")
    args = parser.parse_args()

    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind((args.interface,))
    motors = {motor_id: Motor() for motor_id in range(1, 7)}
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    bus.settimeout(0.2)
    print(f"Pi Arm vcan emulator listening on {args.interface}")

    while running:
        try:
            raw = bus.recv(CAN_FRAME.size)
        except TimeoutError:
            continue
        can_id, length, payload = CAN_FRAME.unpack(raw)
        motor_id = (can_id & 0x7FF) - CAN_BASE
        if length != 8 or motor_id not in motors:
            continue
        reply = response(motors[motor_id], payload)
        if reply is not None:
            bus.send(CAN_FRAME.pack(CAN_BASE + motor_id, 8, reply))

    bus.close()


if __name__ == "__main__":
    main()
