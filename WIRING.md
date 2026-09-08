# Wiring Guide — BTS7960 Motor Driver(s) + HC-SR04 Ultrasonic Sensor(s)
### Raspberry Pi 5

This goes with `motor_ultrasonic_control.py`. It covers a single
motor + single sensor setup, and how to scale up to 2 or 4 motors and
2, 3, or 4 sensors using the same pin map.

---

## 1. Parts list

- Raspberry Pi 5
- 1, 2, or 4x **BTS7960** (a.k.a. IBT-2) H-bridge motor driver modules
- 1, 2, 3, or 4x **HC-SR04** ultrasonic distance sensors
- 1x separate motor power supply (battery pack) matching your motor voltage — **do not power motors from the Pi**
- Per HC-SR04 ECHO pin: 1x 1kΩ resistor + 1x 2kΩ resistor (voltage divider — see §4)
- Common ground bus/rail connecting Pi GND, motor driver GND(s), sensor GND(s), and battery GND

---

## 2. BTS7960 module pins

Each BTS7960 board breaks out:

| Label | Meaning | Connects to |
|---|---|---|
| `RPWM` | Forward speed PWM input | Pi GPIO (3.3V logic, driver accepts 3.3–5V) |
| `LPWM` | Reverse speed PWM input | Pi GPIO |
| `R_EN` | Right-side enable | Pi GPIO |
| `L_EN` | Left-side enable | Pi GPIO |
| `VCC` | Logic power (5V) | Pi 5V pin |
| `GND` | Logic/power ground | Common ground |
| `B+` / `B-` | Motor output | Motor terminals |
| `+` / `-` (power screw terminals) | Motor supply power | External battery pack |

BTS7960's `RPWM`/`LPWM`/`R_EN`/`L_EN` pins are TTL-level and read 3.3V
from the Pi as a valid HIGH, so **no level shifting is needed on the
motor driver side** — only the HC-SR04 `ECHO` line needs it (§4).

---

## 3. Suggested full pin map (BCM numbering)

This map lets you populate **up to 4 motors AND up to 4 sensors at the
same time** without any pin conflicts. If you're only using 1 motor
and 1 sensor, you only need the first row of each table — that's
exactly what ships uncommented in the script.

### Motor drivers

| Driver | RPWM | LPWM | R_EN | L_EN |
|---|---|---|---|---|
| motor1 | GPIO12 | GPIO13 | GPIO20 | GPIO21 |
| motor2 | GPIO18 | GPIO19 | GPIO16 | GPIO26 |
| motor3 | GPIO5 | GPIO6 | GPIO22 | GPIO27 |
| motor4 | GPIO17 | GPIO4 | GPIO25 | GPIO24 |

### Ultrasonic sensors

| Sensor | TRIG | ECHO |
|---|---|---|
| front | GPIO23 | GPIO24* |
| rear | GPIO15 | GPIO14 |
| left | GPIO7 | GPIO8 |
| right | GPIO10 | GPIO9 |

\* **Note:** GPIO24 is shared between `motor4`'s `L_EN` and the
`front` sensor's `ECHO` in the *maximum* 4-motor + 4-sensor layout —
that's the one collision in a fully-populated system (26 usable GPIO
pins isn't quite enough for 24 pins of motors+sensors plus zero
reuse). If you populate all 4 motors AND all 4 sensors, change the
`front` sensor's `echo` to GPIO2 (frees up I2C, otherwise unused here)
in the script's `SENSORS` list. For anything ≤ 3 motors or ≤ 3
sensors combined with the other maxed out, the table above has no
conflicts.

This uses GPIO2/3 (I2C), GPIO14/15 (UART), and GPIO7/8/9/10/11 (SPI)
as plain GPIO. That's fine as long as you're not also using I2C,
UART, or SPI peripherals on this Pi. If you are, drop back to fewer
motors/sensors and keep those buses free.

---

## 4. HC-SR04 wiring — the important part: ECHO needs a voltage divider

HC-SR04 runs its logic at **5V**, but Raspberry Pi GPIO pins are
**3.3V-only inputs** — feeding 5V into a Pi GPIO can damage it. The
`TRIG` pin is fine to drive directly from the Pi (Pi's 3.3V output
still reads as a valid HIGH for the sensor). The `ECHO` *output* from
the sensor is the one you must step down.

Use a simple resistive voltage divider on ECHO:

```
 HC-SR04                                     Raspberry Pi
 ECHO ●──────┬───────[ 1 kΩ ]───────┬────────● GPIOxx (ECHO in)
             │                      │
           (5V pulse)             [ 2 kΩ ]
                                     │
                                     ●
                                    GND
```

- `ECHO → 1kΩ → node → GPIO pin`
- `node → 2kΩ → GND`

This divides the 5V pulse down to **5V × 2kΩ/(1kΩ+2kΩ) ≈ 3.33V**,
which is a safe HIGH for the Pi.

Full per-sensor wiring:

| HC-SR04 pin | Connects to |
|---|---|
| `VCC` | Pi 5V |
| `TRIG` | Pi GPIO (direct, no divider needed) |
| `ECHO` | Through 1kΩ/2kΩ divider (above) → Pi GPIO |
| `GND` | Common ground |

---

## 5. Power

- **Never power motors from the Pi's 5V rail.** Use a separate
  battery/power supply sized for your motors, sharing only a common
  ground with the Pi.
- Each BTS7960's logic `VCC` can come from the Pi's 5V pin if your
  total logic-side current draw stays modest (a few BTS7960 boards'
  logic sections draw very little current — it's the motor side that's
  heavy, and that's on the separate battery). If in doubt, power logic
  from a separate regulated 5V source too.
- HC-SR04 `VCC` also needs 5V — same Pi 5V rail as the driver logic is
  fine.
- Tie **all** grounds together: Pi GND, each BTS7960 GND, each
  HC-SR04 GND, and the motor battery's GND. Without a common ground,
  the PWM/trigger signals won't have a valid reference and readings
  or motor control will be erratic.

---

## 6. Scaling up/down — what actually changes

| Config | What to edit |
|---|---|
| 1 motor, 1 sensor (default) | Nothing — ships ready to go |
| 2 motors | Uncomment `motor2` line in `MOTORS` list, wire per table above |
| 4 motors | Uncomment `motor2`–`motor4` lines, wire per table above |
| 2–4 sensors | Uncomment the matching `UltrasonicConfig` lines in `SENSORS`, wire each with its own voltage divider |
| Different GPIO numbers | Just edit the pin numbers in `MotorConfig`/`UltrasonicConfig` — the rest of the script loops over the lists automatically |
| Motor spins the wrong way | Set `invert=True` on that motor's `MotorConfig` instead of re-wiring |

No other code changes are required — `RobotController` in
`motor_ultrasonic_control.py` sets up and drives whatever is in the
`MOTORS`/`SENSORS` lists, however many entries that is.

---

## 7. Before you power anything on

1. Run `gpiodetect` and confirm which chip is `pinctrl-rp1`/`rp1` —
   update `GPIO_CHIP` in the script if it isn't 4 on your image.
2. Wire and double-check the ECHO voltage divider on every sensor
   *before* connecting VCC — a direct 5V→GPIO connection can damage
   the Pi.
3. With motors' battery pack **disconnected**, run:
   `python3 motor_ultrasonic_control.py --mode status`
   and confirm sensor readings look sane (walk your hand toward/away
   from each sensor and watch the numbers change).
4. Only then connect the motor battery and try `--mode ramp` with the
   robot up on blocks (wheels off the ground) before `--mode obstacle`.
