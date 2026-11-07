# Marble ground test

Runs the motor tests of the Control ground test one at a time on a QuadPlane
with DShot ESCs and ESC telemetry, and logs the results under the Control field
keys. Direction and structure checks stay with the operator, who watches during
the hold.

## Tests

| GT_TEST | Test | What the script does |
|---|---|---|
| 1 | Individual lift motor | Drives Motor`GT_MOTOR` alone up to 16500 RPM in a closed loop, then holds for 10 s |
| 2 | Shared lift motors | Drives Motor1-4 on one PWM until their mean is 13000 RPM, then holds for 90 s and records the hottest lift ESC 45, 60 and 90 s after reaching the target |
| 3 | Cruise motor max | Ramps the Throttle output to `GT_THR_PCT` (80%) of the DShot range, then holds for 3 s |
| 4 | Cruise motor sustained | Drives the Throttle output to 7800 RPM in a closed loop, then holds for 180 s |

`GT_RPM` and `GT_HOLD` override a test's target and hold time when they are
not 0. The individual lift motors follow the Control table: 1 front right,
2 back left, 3 front left, 4 back right.

The script overrides the motor outputs' PWM directly, so the servo functions
stay as they are for flight (Motor1-4 on 33-36, cruise on 70 Throttle). Every
motor it is not testing is held at 1000 for the whole run.

## Running a test

1. Set `SCR_ENABLE` 1, copy the script to `APM/scripts` and reboot.
2. Set an `RCx_OPTION` to 300 (Scripting1) on a 3-position switch, and
   `SERVO_BLH_POLES` so ESC telemetry reports mechanical RPM.
3. Set `GT_TEST` (and `GT_MOTOR` for test 1).
4. With the switch low, select MANUAL, keep the throttle stick low and arm.
5. Move the switch to high. The test starts only when the switch moves to high
   while armed. A switch already high when the script loads, or left high
   after an abort, has to be moved away first.
6. Keep the switch high for the whole test. When the test ends, the motors
   ramp down and the aircraft disarms.

## Aborts

Every abort disarms at once. The motor overrides lapse within 60 ms if the
script stops, and DShot outputs send zero while disarmed. Each abort is sent as
a critical GCS message and logged.

The test aborts immediately when any of these happen:

- The switch leaves high (middle or low)
- The aircraft is disarmed
- The mode leaves MANUAL
- Motor emergency stop is active (a script override bypasses the E-stop, so the
  script checks it itself)
- RC input is lost
- The throttle stick is raised above 10%
- `GT_TEST` changes
- A tested ESC's telemetry is older than 500 ms
- The script itself errors

These limits abort after holding for 0.5 s:

- A tested ESC is above `GT_ESC_TMAX`
- Battery current is above `GT_CURR_MAX`
- Battery voltage is below `GT_VOLT_MIN` (`BATT_LOW_VOLT` when that is 0)

These abort after the time shown:

- A tested motor is over the target by `GT_RPM_OVR`% (0.3 s)
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

An aborted run logs a `GTRS` row whose `Key` starts with `abort:`.

`GTST` logs the run at 10 Hz: test, state (1 ramp, 2 hold, 3 ramp down,
4 off), target, commanded PWM, mean tested RPM, battery current, summed tested
ESC current, battery voltage and the hottest tested ESC. `GT_RPM` and `GT_PWM`
are also sent as named floats for the GCS.

The transition check is not automated yet.
