#!/usr/bin/env python3
"""
motor_ultrasonic_control.py

Configurable control script for:
  - 1, 2, or 4x BTS7960 motor drivers
  - 1, 2, 3, or 4x HC-SR04 ultrasonic distance sensors

Target: Raspberry Pi 5, using lgpio (works with Pi 5's rp1 GPIO chip).

HOW TO ADD/REMOVE HARDWARE
---------------------------
Everything is driven by two lists near the top of this file:

    MOTORS   -> one MotorConfig entry per BTS7960 driver
    SENSORS  -> one UltrasonicConfig entry per HC-SR04 sensor

To go from 1 motor to 2 or 4, just add more MotorConfig(...) lines.
To go from 1 sensor to 2, 3, or 4, just add more UltrasonicConfig(...) lines.
Nothing else in the script needs to change -- all loops iterate over
whatever is in these two lists.

See WIRING.md (generated alongside this script) for full pinout
tables, a wiring diagram, and the ECHO-pin voltage divider you need
because HC-SR04 outputs 5V logic and the Pi's GPIO is 3.3V-only.

USAGE
-----
    python3 motor_ultrasonic_control.py --mode status         # read sensors only, motors untouched (default, safest)
    python3 motor_ultrasonic_control.py --mode ramp           # ramp all motors fwd/back, sensors ignored
    python3 motor_ultrasonic_control.py --mode obstacle        # drive forward, auto-stop/back off on obstacle
    python3 motor_ultrasonic_control.py --mode obstacle --speed 40 --stop-cm 25
"""

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Dict

import lgpio

# ============================================================================
# CONFIGURATION -- EDIT THIS SECTION FOR YOUR HARDWARE
# ============================================================================

# GPIO chip number. Check with `gpiodetect` -- look for "pinctrl-rp1"/"rp1".
GPIO_CHIP = 4

# PWM frequency for motor speed control (Hz). 1000 Hz is stable for DC motors.
PWM_FREQ = 1000

# Ambient temperature in Celsius, used to temperature-compensate the speed
# of sound for more accurate ultrasonic distance readings. Set this to your
# actual room/outdoor temperature if you want best accuracy; 20 C is a fine
# default indoors.
TEMPERATURE_C = 20.0

# How long to wait for an echo before giving up on a single reading (sec).
# 0.03 s corresponds to a round trip of ~5 m at 343 m/s, comfortably above
# the HC-SR04's usable range (~4 m).
ECHO_TIMEOUT_S = 0.03

# Minimum time to wait between triggering the SAME sensor twice, and between
# triggering DIFFERENT sensors in the same polling pass. The HC-SR04
# datasheet recommends >= 60 ms between measurements to let ultrasonic echoes
# fully die out and avoid one sensor hearing another's echo.
SENSOR_SETTLE_S = 0.06

# Distance (cm) at which the "obstacle" demo mode will stop/back off.
OBSTACLE_STOP_CM = 20.0


@dataclass
class MotorConfig:
    name: str
    rpwm: int   # forward PWM pin (BCM)
    lpwm: int   # reverse PWM pin (BCM)
    r_en: int   # right-side enable pin (BCM)
    l_en: int   # left-side enable pin (BCM)
    invert: bool = False  # set True if the motor spins backwards for a given sign


@dataclass
class UltrasonicConfig:
    name: str
    trig: int   # trigger pin (BCM)
    echo: int   # echo pin (BCM) -- MUST go through a voltage divider, see WIRING.md
    max_distance_cm: float = 400.0  # readings beyond this are treated as invalid/None


# --- Motor driver list -------------------------------------------------
# Start with just the first entry for a single-driver robot. Uncomment/add
# more MotorConfig lines for a 2-motor (differential drive) or 4-motor
# (4WD / mecanum) robot. Pin numbers below match WIRING.md's suggested map.
MOTORS: List[MotorConfig] = [
    MotorConfig(name="motor1", rpwm=12, lpwm=13, r_en=20, l_en=21),
    # MotorConfig(name="motor2", rpwm=18, lpwm=19, r_en=16, l_en=26),
    # MotorConfig(name="motor3", rpwm=5,  lpwm=6,  r_en=22, l_en=27),
    # MotorConfig(name="motor4", rpwm=17, lpwm=4,  r_en=25, l_en=24),
]

# --- Ultrasonic sensor list ---------------------------------------------
# Start with just "front". Uncomment/add more for 2, 3, or 4 sensors
# (e.g. front/rear, or front/rear/left/right). Pin numbers below match
# WIRING.md's suggested map and avoid the pins used by all 4 motors above,
# so you can populate both lists fully at the same time if needed.
SENSORS: List[UltrasonicConfig] = [
    UltrasonicConfig(name="front", trig=23, echo=24),
    # UltrasonicConfig(name="rear",  trig=15, echo=14),
    # UltrasonicConfig(name="left",  trig=7,  echo=8),
    # UltrasonicConfig(name="right", trig=10, echo=9),
]

# ============================================================================
# DRIVER CLASSES -- shouldn't need editing below this line
# ============================================================================


class MotorDriver:
    """One BTS7960 half-bridge driver (one motor)."""

    def __init__(self, handle: int, cfg: MotorConfig):
        self.h = handle
        self.cfg = cfg
        self._speed = 0

    def setup(self):
        lgpio.gpio_claim_output(self.h, self.cfg.r_en, 0)
        lgpio.gpio_claim_output(self.h, self.cfg.l_en, 0)
        lgpio.gpio_claim_output(self.h, self.cfg.rpwm, 0)
        lgpio.gpio_claim_output(self.h, self.cfg.lpwm, 0)
        lgpio.gpio_write(self.h, self.cfg.r_en, 1)
        lgpio.gpio_write(self.h, self.cfg.l_en, 1)
        lgpio.tx_pwm(self.h, self.cfg.rpwm, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, self.cfg.lpwm, PWM_FREQ, 0)

    def set_speed(self, speed: int):
        """speed: -100 (full reverse) to 100 (full forward). Dead-band at +-5."""
        speed = max(-100, min(100, speed))
        if self.cfg.invert:
            speed = -speed
        self._speed = speed
        duty = abs(speed)

        if speed >= 5:
            lgpio.tx_pwm(self.h, self.cfg.lpwm, PWM_FREQ, 0)
            lgpio.tx_pwm(self.h, self.cfg.rpwm, PWM_FREQ, duty)
        elif speed <= -5:
            lgpio.tx_pwm(self.h, self.cfg.rpwm, PWM_FREQ, 0)
            lgpio.tx_pwm(self.h, self.cfg.lpwm, PWM_FREQ, duty)
        else:
            lgpio.tx_pwm(self.h, self.cfg.rpwm, PWM_FREQ, 0)
            lgpio.tx_pwm(self.h, self.cfg.lpwm, PWM_FREQ, 0)

    def stop(self):
        self.set_speed(0)

    def cleanup(self):
        try:
            lgpio.tx_pwm(self.h, self.cfg.rpwm, PWM_FREQ, 0)
            lgpio.tx_pwm(self.h, self.cfg.lpwm, PWM_FREQ, 0)
            lgpio.gpio_write(self.h, self.cfg.r_en, 0)
            lgpio.gpio_write(self.h, self.cfg.l_en, 0)
            for pin in (self.cfg.rpwm, self.cfg.lpwm, self.cfg.r_en, self.cfg.l_en):
                lgpio.gpio_free(self.h, pin)
        except Exception as e:
            print(f"  (cleanup warning for {self.cfg.name}: {e})")


class UltrasonicSensor:
    """One HC-SR04 (or compatible) ultrasonic distance sensor."""

    def __init__(self, handle: int, cfg: UltrasonicConfig):
        self.h = handle
        self.cfg = cfg

    def setup(self):
        lgpio.gpio_claim_output(self.h, self.cfg.trig, 0)
        lgpio.gpio_claim_input(self.h, self.cfg.echo)

    def measure_cm(self, temperature_c: float = TEMPERATURE_C,
                   timeout: float = ECHO_TIMEOUT_S) -> Optional[float]:
        """
        Trigger a single ping and return distance in cm, or None if the
        sensor timed out (no echo / obstacle out of range).

        Calculation: distance = (echo_pulse_duration * speed_of_sound) / 2
        The /2 accounts for the pulse traveling to the obstacle AND back.
        Speed of sound is temperature-compensated:
            v (m/s) = 331.3 + 0.606 * temperature_C
        """
        # 10 microsecond trigger pulse, per HC-SR04 datasheet.
        lgpio.gpio_write(self.h, self.cfg.trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.h, self.cfg.trig, 0)

        # Wait for echo pin to go HIGH (start of return pulse).
        t0 = time.perf_counter()
        while lgpio.gpio_read(self.h, self.cfg.echo) == 0:
            if time.perf_counter() - t0 > timeout:
                return None  # no echo start -- sensor not responding
        pulse_start = time.perf_counter()

        # Wait for echo pin to go LOW again (end of return pulse).
        while lgpio.gpio_read(self.h, self.cfg.echo) == 1:
            if time.perf_counter() - pulse_start > timeout:
                return None  # echo stuck high -- likely out of range
        pulse_end = time.perf_counter()

        duration_s = pulse_end - pulse_start
        speed_of_sound_m_s = 331.3 + 0.606 * temperature_c
        distance_cm = (duration_s * speed_of_sound_m_s * 100.0) / 2.0

        if distance_cm <= 0 or distance_cm > self.cfg.max_distance_cm:
            return None
        return round(distance_cm, 1)

    def cleanup(self):
        try:
            lgpio.gpio_free(self.h, self.cfg.trig)
            lgpio.gpio_free(self.h, self.cfg.echo)
        except Exception as e:
            print(f"  (cleanup warning for {self.cfg.name}: {e})")


class RobotController:
    """Owns the GPIO handle and all configured motors + sensors."""

    def __init__(self, motor_configs: List[MotorConfig], sensor_configs: List[UltrasonicConfig]):
        self.h = None
        self.motor_configs = motor_configs
        self.sensor_configs = sensor_configs
        self.motors: List[MotorDriver] = []
        self.sensors: List[UltrasonicSensor] = []

    def setup(self):
        print(f"Opening gpiochip{GPIO_CHIP}...")
        self.h = lgpio.gpiochip_open(GPIO_CHIP)

        print(f"Setting up {len(self.motor_configs)} motor driver(s): "
              f"{[m.name for m in self.motor_configs]}")
        for cfg in self.motor_configs:
            m = MotorDriver(self.h, cfg)
            m.setup()
            self.motors.append(m)

        print(f"Setting up {len(self.sensor_configs)} ultrasonic sensor(s): "
              f"{[s.name for s in self.sensor_configs]}")
        for cfg in self.sensor_configs:
            s = UltrasonicSensor(self.h, cfg)
            s.setup()
            self.sensors.append(s)

        print("Setup complete.")

    def set_all_motors(self, speed: int):
        for m in self.motors:
            m.set_speed(speed)

    def stop_all_motors(self):
        for m in self.motors:
            m.stop()

    def read_all_distances(self) -> Dict[str, Optional[float]]:
        """
        Reads every configured sensor ONE AT A TIME (never simultaneously),
        with a settle delay between each, so sensors don't pick up each
        other's echoes. Returns {sensor_name: distance_cm_or_None}.
        """
        readings: Dict[str, Optional[float]] = {}
        for s in self.sensors:
            readings[s.cfg.name] = s.measure_cm()
            time.sleep(SENSOR_SETTLE_S)
        return readings

    def cleanup(self):
        print("Cleaning up...")
        for m in self.motors:
            m.cleanup()
        for s in self.sensors:
            s.cleanup()
        if self.h is not None:
            try:
                lgpio.gpiochip_close(self.h)
            except Exception as e:
                print(f"  (chip close warning: {e})")
        print("GPIO cleaned up.")


# ============================================================================
# DEMO MODES
# ============================================================================


def mode_status(robot: RobotController, duration_s: float = 20.0):
    """Read-only: prints sensor distances in a loop. Motors are never driven.
    Good first test to confirm wiring/pin numbers before moving anything."""
    print(f"--- STATUS MODE (read-only, {duration_s:.0f}s) ---")
    print("Motors will NOT move. Press Ctrl+C to stop early.\n")
    end = time.time() + duration_s
    while time.time() < end:
        readings = robot.read_all_distances()
        line = "  ".join(
            f"{name}: {'--' if dist is None else f'{dist:5.1f} cm'}"
            for name, dist in readings.items()
        )
        print(line, end="\r")
    print("\nDone.")


def mode_ramp(robot: RobotController):
    """Ramps all configured motors forward then backward together.
    Sensors are read but only printed, not acted on."""
    print("--- RAMP MODE ---")
    print("Make sure motors are secure!")
    time.sleep(2)

    for direction, label in ((1, "FORWARD"), (-1, "REVERSE")):
        print(f"\n--- Ramping {label} ---")
        for s in range(0, 101, 5):
            robot.set_all_motors(s * direction)
            print(f"Speed: {s}%   ", end="\r")
            time.sleep(0.1)
        time.sleep(1)
        for s in range(100, -1, -5):
            robot.set_all_motors(s * direction)
            print(f"Speed: {s}%   ", end="\r")
            time.sleep(0.1)
        robot.stop_all_motors()
        time.sleep(1)
    print("\nRamp test complete.")


def mode_obstacle(robot: RobotController, speed: int, stop_cm: float):
    """Drives forward; if ANY sensor sees an obstacle closer than stop_cm,
    stops, briefly reverses, then keeps polling. This is a simple
    reactive-avoidance demo, not a full navigation stack."""
    print("--- OBSTACLE-AVOIDANCE MODE ---")
    print(f"Base speed: {speed}%   Stop distance: {stop_cm} cm")
    print("Press Ctrl+C to stop.\n")

    if not robot.sensors:
        print("No sensors configured -- nothing to avoid, just driving forward.")

    try:
        while True:
            readings = robot.read_all_distances()
            valid = [d for d in readings.values() if d is not None]
            closest = min(valid) if valid else None

            status = "  ".join(
                f"{name}: {'--' if d is None else f'{d:5.1f} cm'}"
                for name, d in readings.items()
            )

            if closest is not None and closest < stop_cm:
                print(f"{status}  -> OBSTACLE at {closest:.1f} cm, backing off ", end="\r")
                robot.stop_all_motors()
                time.sleep(0.2)
                robot.set_all_motors(-speed)
                time.sleep(0.4)
                robot.stop_all_motors()
                time.sleep(0.2)
            else:
                print(f"{status}  -> clear, driving forward     ", end="\r")
                robot.set_all_motors(speed)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        robot.stop_all_motors()


# ============================================================================
# MAIN
# ============================================================================


def parse_args():
    p = argparse.ArgumentParser(description="Configurable BTS7960 + HC-SR04 control for Raspberry Pi 5")
    p.add_argument("--mode", choices=["status", "ramp", "obstacle"], default="status",
                    help="status = read sensors only (safe default); ramp = motor speed sweep test; "
                         "obstacle = drive forward with auto-stop on obstacle")
    p.add_argument("--speed", type=int, default=40, help="base speed (0-100) for obstacle mode")
    p.add_argument("--stop-cm", type=float, default=OBSTACLE_STOP_CM,
                    help="obstacle-stop distance in cm for obstacle mode")
    p.add_argument("--status-seconds", type=float, default=20.0,
                    help="how long to run status mode, in seconds")
    return p.parse_args()


def main():
    args = parse_args()
    robot = RobotController(MOTORS, SENSORS)
    try:
        robot.setup()
        time.sleep(0.5)

        if args.mode == "status":
            mode_status(robot, duration_s=args.status_seconds)
        elif args.mode == "ramp":
            mode_ramp(robot)
        elif args.mode == "obstacle":
            mode_obstacle(robot, speed=args.speed, stop_cm=args.stop_cm)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
        raise
    finally:
        robot.cleanup()


if __name__ == "__main__":
    main()
