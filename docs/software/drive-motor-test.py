# To be run on a Raspberry Pi with pigpio installed and the pigpio daemon running.
import pigpio
import time

GPIO_PIN = 18  # PWM output pin

# PWM pulse widths in microseconds (specific to the ESC)
MIN_PW = 800
MID_PW = 1500
MAX_PW = 2200

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")

pi.set_mode(GPIO_PIN, pigpio.OUTPUT)

def set_pulsewidth(us):
    us = max(MIN_PW, min(MAX_PW, us))
    pi.set_servo_pulsewidth(GPIO_PIN, us)
    print(f"Pulse width set to {us} µs")

try:
    print("ESC PWM Test")
    print("Enter pulse width in microseconds")
    print(f"Range: {MIN_PW}–{MAX_PW}, midpoint {MID_PW}")
    print("Type 'mid', 'min', 'max', or 'q' to quit\n")

    set_pulsewidth(MID_PW)
    time.sleep(1)

    while True:
        cmd = input("PWM> ").strip().lower()

        if cmd == "q":
            break
        elif cmd == "mid":
            set_pulsewidth(MID_PW)
        elif cmd == "min":
            set_pulsewidth(MIN_PW)
        elif cmd == "max":
            set_pulsewidth(MAX_PW)
        else:
            try:
                pw = int(cmd)
                set_pulsewidth(pw)
            except ValueError:
                print("Invalid input")

finally:
    print("Stopping ESC")
    pi.set_servo_pulsewidth(GPIO_PIN, MID_PW)
    time.sleep(0.5)
    pi.stop()
