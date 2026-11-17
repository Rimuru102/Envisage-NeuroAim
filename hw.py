"""Real turret over serial. Same line format as SimTurret."""
import time
import serial
from config import YAW_MIN_DEG, YAW_MAX_DEG, PITCH_MIN_DEG, PITCH_MAX_DEG

def fmt(line: bytes) -> bytes:
    """Clamp an A line to the config limits and force fire to 0."""
    _, y, p, _ = line.decode().strip().split(",")
    y = max(YAW_MIN_DEG * 100, min(YAW_MAX_DEG * 100, int(y)))
    p = max(PITCH_MIN_DEG * 100, min(PITCH_MAX_DEG * 100, int(p)))
    return f"A,{y},{p},0\n".encode()

class HwTurret:
    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=0, write_timeout=0.1)
        time.sleep(2.0)                      # the ESP32 resets when the port opens
        self.s.reset_input_buffer()          # drop the boot "Ready"; any later one means the board restarted
        self.resets = 0
    def write(self, line: bytes):
        if b"Ready" in self.s.read(self.s.in_waiting):
            self.resets += 1
            print(f"WARNING: ESP32 restarted (#{self.resets}): power dip or loose wire? The turret re-centered.")
        self.s.write(fmt(line))
    def close(self):
        self.s.write(b"A,0,0,0\n"); self.s.close()   # center, then release

if __name__ == "__main__":
    assert fmt(b"A,9999,-5000,1\n") == f"A,{YAW_MAX_DEG*100},{PITCH_MIN_DEG*100},0\n".encode()
    print("hw.py OK")
