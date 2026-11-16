# Marble wing heater

Cycles the wing heater on and off on a timer, controlled from the ground by
parameter writes. The heater switch reads one servo output: above 1900 PWM is
on, below 1100 is off. There is no temperature sensing, so the timing alone
keeps the wing from overheating.

## Setup

1. Set `SCR_ENABLE` 1, copy the script to `APM/scripts` and reboot.
2. On the heater switch's output, set `SERVOn_FUNCTION` 94 (Script1),
   `SERVOn_MIN` 1000, `SERVOn_TRIM` 1000 and `SERVOn_MAX` 2000. The trim is
   what the output sends whenever the script is not turning the heater on, so
   it must be below 1100.
3. Set `WHT_ON_S` and `WHT_OFF_S`.

## Use from the ground

Write `WHT_MODE` 1 to start cycling and 0 to stop, from any GCS. Every `WHT_`
parameter is read each loop, so a change applies at once. A cycle starts with
the heater on. The heater runs armed or disarmed.

`WHT_MODE` is set to 0 when the script loads, so the heater never comes on by
itself at power up.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `WHT_MODE` | 0 | 0 off, 1 cycling |
| `WHT_ON_S` | 10 | Seconds on per cycle, 0 keeps it off |
| `WHT_OFF_S` | 20 | Seconds off per cycle |
| `WHT_SCR_OUT` | 1 | Scripting output used, 1 for Script1 (function 94) |
| `WHT_PWM_ON` | 2000 | PWM sent while on |

Nothing limits the timing, so a `WHT_OFF_S` of 0 leaves the heater on.

## Failing off

- The on command is an override that lapses 300 ms after the script last sent
  it, after which the output returns to `SERVOn_TRIM` (off). If the script
  errors, is stopped or scripting dies, the heater turns off. An on phase can
  therefore run up to 300 ms past `WHT_ON_S`.
- With no output on the chosen scripting function, `WHT_MODE` is set to 0 and
  a warning is sent.
- A flight controller that hangs entirely could leave the output where it was.
  A thermal cut-off in the heater circuit covers that case and the lack of
  temperature sensing.

## Feedback

`WHT_MODE` (the mode) and `WHT_ON` (1 while the heater is on, 0 between
heats) are sent as named floats every second, and `WHT` logs both once a
second. Mode changes are sent as GCS messages. Onboard reads the two named
floats for Control, and takes an aircraft that has not sent them for 5 s as
having no wing heating running.
