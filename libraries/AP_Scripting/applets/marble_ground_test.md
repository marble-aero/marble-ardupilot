# Marble ground test

Runs the motor tests of the Control ground test one at a time on a QuadPlane
with DShot ESCs and ESC telemetry, and logs the results under the Control field
keys. Direction and structure checks stay with the operator, who watches during
the hold.

## Tests

| GT_TEST | Test | What the script does |
|---|---|---|
| 1 | Individual lift motor | Drives Motor`GT_MOTOR` alone up to 16500 RPM in a closed loop, then holds for 5 s |
| 2 | Shared lift motors | Drives Motor1-4 on one PWM until their mean is 13000 RPM, then holds for 90 s and records the hottest lift ESC 45, 60 and 90 s after reaching the target |
| 3 | Cruise motor max | Ramps the Throttle output to `GT_THR_PCT` (80%) of the DShot range, then holds for 3 s |
| 4 | Cruise motor sustained | Drives the Throttle output to 7800 RPM in a closed loop, then holds for 180 s |
| 5 | Transition | Takeoff on the lift motors at 14000 RPM, hands over to the cruise motor at 9000 RPM, runs the cruise motor alone at 7500 RPM for 60 s, then lands back on the lift motors, see [Transition test](#transition-test) |

The targets and hold times are fixed by the Control ground test. The closed
loop tests use an RPM loop gain of 0.02 PWM per second for each RPM of error,
limited by `GT_SLEW`. The individual lift motors follow the Control table:
1 front right, 2 back left, 3 front left, 4 back right.

The script overrides the motor outputs' PWM directly, so the servo functions
stay as they are for flight (Motor1-4 on 33-36, cruise on 70 Throttle). Every
motor it is not testing is held at 1000 for the whole run.

## Parameters

| Parameter | Default | Range | Description |
|---|---|---|---|
| `GT_TEST` | 0 | 0-5 | Test to run when the switch goes high: 0 none, 1 individual lift motor, 2 shared lift motors, 3 cruise motor max, 4 cruise motor sustained, 5 transition |
| `GT_MOTOR` | 1 | 1-4 | Lift motor driven by the individual lift motor test: 1 front right, 2 back left, 3 front left, 4 back right |
| `GT_THR_PCT` | 80 | 0-100 % | Throttle of the cruise motor max test, as a percentage of the DShot range |
| `GT_PWM_MAX` | 1850 | 1000-2000 PWM | Hard ceiling on the PWM the script commands to any motor |
| `GT_SLEW` | 50 | 5-500 PWM/s | Fastest PWM change of the RPM loop and of the cruise motor max ramp. The transition test's timed ramps are not limited by it, and stopping a test always ramps down at 50% of the PWM range per second |
| `GT_RPM_TOL` | 250 | 50-2000 RPM | A target counts as reached once the RPM stays this close to it for 2 s |
| `GT_RPM_OVR` | 10 | 2-50 % | Abort when a tested motor runs this far over its target RPM for 0.3 s |
| `GT_ESC_TMAX` | 85 | 30-150 degC | Abort when a tested ESC is hotter than this for 0.5 s. A test does not start with a tested ESC already at this temperature |
| `GT_CURR_MAX` | 150 | 10-400 A | Abort when the battery current is above this for 0.5 s |
| `GT_VOLT_MIN` | 0 | 0-100 V | Abort when the battery voltage is below this for 0.5 s, 0 to use `BATT_LOW_VOLT`. A test does not start below it |

## Running a test

1. Set `SCR_ENABLE` 1, copy the script to `APM/scripts` and reboot.
2. Set an `RCx_OPTION` to 300 (Scripting1) on a 3-position switch, and
   `SERVO_BLH_POLES` so ESC telemetry reports mechanical RPM. The switch only
   reads high above 1800 PWM.
3. Set `GT_TEST` (and `GT_MOTOR` for test 1).
4. With the switch low, select MANUAL, keep the throttle stick low and arm.
5. Move the switch to high. The test starts only when the switch moves to high
   while armed. A switch already high when the script loads, or left high
   after an abort, has to be moved away first.
6. Keep the switch high for the whole test. When the test ends, the motors
   ramp down and the aircraft disarms.

While idle the script reports the selected test, arming, and every switch
position change with its PWM, so a test that does not start says why.

## Transition test

The transition test replaces the manual procedure. The servo output changes
(lift motors on one output, cruise motor on RCIN6) and the control wheels are
no longer needed: the script finds the Motor1-4 and Throttle outputs itself
and drives the lift motors on one PWM and the cruise motor on another. Run it
as in [Running a test](#running-a-test) with `GT_TEST` 5.

The lift and cruise motors run together during takeoff and landing, so the
battery current can go above the 150 A `GT_CURR_MAX` default. Check the
current during the first run and raise `GT_CURR_MAX` if needed.

The script steps through these phases and sends a GCS message at each step:

| Phase | Lift motors | Cruise motor | Moves on when |
|---|---|---|---|
| 1 Lift up | Closed loop to 14000 RPM | Off | Lift reaches 14000 RPM |
| 2 Takeoff | Held at 14000 RPM | Off for the first 20 s, then closed loop to 9000 RPM | Cruise has been at 9000 RPM for 1 s |
| 3 Lift down | Ramps linearly to off over 10 s | Held at 9000 RPM until 5 s after reaching it, then closed loop to 7500 RPM | Lift is off and cruise reaches 7500 RPM |
| 4 Cruise | Off | Held at 7500 RPM for 60 s | The hold ends, still in MANUAL |
| 5 Landing | Ramps linearly over 10 s to the PWM that held 14000 RPM in takeoff | Ramps linearly to off over the same 10 s | Both ramps end |
| 6 Landing hold | Closed loop at 14000 RPM, held 15 s from reaching it | Off | The hold ends |
| 7 Lift down | Ramps linearly to off over 10 s | Off | Lift is off |

The aircraft then disarms. An abort in any phase still ramps down at 50% per
second.

A motor counts as at its target the first time it comes within `GT_RPM_TOL`.
The test aborts if the lift or cruise motor does not reach a target within
60 s. Leaving MANUAL aborts in any phase, and the stall check aborts if the
cruise motor stops while it is driven.

## Aborts

An abort ramps every tested motor down at 50% of the PWM range per second
(full to off in 2 s), holds the motors at off for 1 s and then disarms. Once
the ramp down has started, only a disarm or motor emergency stop ends it early.
Each abort is sent as a critical GCS message and logged.

These stop the motors at once, without a ramp down:

- The aircraft is disarmed. DShot outputs send zero while disarmed, so the
  script cannot ramp them.
- Motor emergency stop is active. A script override can bypass E-stop on the
  lift motors, so the script checks it itself and sets every motor to 1000.
- The script itself errors. The motor overrides lapse within 60 ms if the
  script stops.

These ramp down as soon as they happen:

- The switch leaves high (middle or low)
- The mode leaves MANUAL
- RC input is lost
- The throttle stick is raised above 10%
- `GT_TEST` changes
- A tested ESC's telemetry is older than 500 ms

These limits ramp down after holding for 0.5 s:

- A tested ESC is above `GT_ESC_TMAX`
- Battery current is above `GT_CURR_MAX`
- Battery voltage is below `GT_VOLT_MIN` (`BATT_LOW_VOLT` when that is 0)

These ramp down after the time shown:

- A tested motor is over its target by `GT_RPM_OVR`% (0.3 s)
- A tested motor is under 300 RPM above 1150 PWM (1 s)
- The target is not reached within 60 s

The commanded PWM never goes above `GT_PWM_MAX`.

Keep a motor emergency stop switch and a physical battery disconnect as well.
The script is not the only safety layer.

## Results

`GTRS` is logged once per result. `Key` is the Control ground test field key,
for example `individual_lift_motor.front_left.rcout` or
`shared_lift_motor.temp_45s.result`. Means are taken over the hold. A few keys
have no Control field yet:

- `individual_lift_motor.<row>.esc_current_a`, the tested ESC's own current
  next to the battery current in `current_a`
- `shared_lift_motor.starting_esc_temp_range.min` and `.max`

The transition test logs `transition.lift_esc_max_temp.result` and
`transition.cruise_esc_max_temp.result`, the hottest lift and cruise ESC over
the whole test.

An aborted run logs a `GTRS` row whose `Key` starts with `abort:`.

`GTST` logs the run at 10 Hz:

| Field | Meaning |
|---|---|
| `Tst` | Test |
| `St` | State: 1 ramp, 2 hold, 3 ramp down, 4 off. The transition test stays in 1 until its lift is off, or in 3 after an abort |
| `Ph` | Transition phase (1-7), 0 for the other tests |
| `Tgt` | Target RPM, or target PWM for the cruise motor max test |
| `PWM`, `RPM` | Commanded PWM and mean RPM of the tested motors, the lift motors in the transition test |
| `CPWM`, `CRPM` | Cruise motor commanded PWM and RPM in the transition test, 0 otherwise |
| `BCur` | Battery current |
| `ECur` | Summed current of the tested ESCs |
| `Volt` | Battery voltage |
| `ETmp` | Hottest tested ESC |

`GT_RPM` and `GT_PWM` are also sent as named floats for the GCS, plus
`GT_CRPM` and `GT_CPWM` in the transition test.
