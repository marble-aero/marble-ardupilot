--[[
  Marble ground test automation for the QuadPlane motor tests

  Runs one motor test of the Control ground test at a time, chosen with GT_TEST:
    1 individual lift motor (GT_MOTOR), closed loop on ESC RPM
    2 shared lift motors, all four on one PWM, closed loop on their mean RPM
    3 cruise motor max, open loop to GT_THR_PCT
    4 cruise motor sustained, closed loop on ESC RPM
    5 transition, takeoff on lift, over to cruise, cruise alone, then land on lift

  The operator arms in MANUAL with the throttle stick low, then moves the
  Scripting1 switch (RCx_OPTION 300) to high to start. Anything but high
  aborts. The script drives the motor outputs directly by overriding their
  PWM with a short timeout, so no SERVOn_FUNCTION changes are needed and the
  outputs fall back on their own if the script stops. Every abort, and the end
  of every test, ramps the motors down at 50% per second and then disarms,
  which makes the DShot outputs send zero. A disarm or motor emergency stop
  stops the motors at once, without a ramp down.

  Results are logged as GTRS messages keyed by the Control ground test field
  key, and the run is logged at 10 Hz as GTST.
--]]

local PARAM_TABLE_KEY = 61
local PARAM_TABLE_PREFIX = "GT_"

local OVERRIDE_TIMEOUT_MS = 60

local MODE_MANUAL = 0
local AUX_SCRIPTING1 = 300
local SWITCH_HIGH = 2

local PWM_OFF = 1000
local PWM_FULL = 2000
local LIFT_MOTOR_FUNCTIONS = { 33, 34, 35, 36 }
local CRUISE_MOTOR_FUNCTION = 70
local LIFT_ROW_KEYS = { "front_right", "back_left", "front_left", "back_right" }

local STICK_LOW_FRACTION = 0.1
local LIMIT_PERSIST_MS = 500
local STALL_PWM = 1150
local STALL_RPM = 300
local RAMP_TIMEOUT_MS = 60000
local RAMP_DOWN_PWM_S = (PWM_FULL - PWM_OFF) * 0.5 -- 50% per second, full to off in 2 s

-- enums are tables, the embedded Lua allows only 100 locals per function
local TEST = {
  INDIVIDUAL_LIFT = 1,
  SHARED_LIFT = 2,
  CRUISE_MAX = 3,
  CRUISE_SUSTAINED = 4,
  TRANSITION = 5,
}

local SECTION_KEYS = {
  [TEST.INDIVIDUAL_LIFT] = "individual_lift_motor",
  [TEST.SHARED_LIFT] = "shared_lift_motor",
  [TEST.CRUISE_MAX] = "cruise_motor_max",
  [TEST.CRUISE_SUSTAINED] = "cruise_motor_sustained",
  [TEST.TRANSITION] = "transition",
}

-- transition test targets and timings, and the phases it steps through in order
local TR = {
  LIFT_RPM = 14000,
  CRUISE_RPM = 10000,
  CRUISE_LOW_RPM = 7500,
  LIFT_HOLD_MS = 20000,    -- lift alone at LIFT_RPM before the cruise motor starts
  CRUISE_LEAD_MS = 1000,   -- cruise at CRUISE_RPM before the lift ramps down
  CRUISE_HIGH_MS = 5000,   -- cruise at CRUISE_RPM before it drops to CRUISE_LOW_RPM
  LIFT_DOWN_MS = 10000,
  LANDING_RAMP_MS = 10000, -- lift ramps up and cruise ramps to off together
  LANDING_HOLD_MS = 15000, -- lift alone at LIFT_RPM, from when it reaches it
  LANDING_DOWN_MS = 10000,
}
local TR_PHASE = {
  LIFT_UP = 1,      -- lift to LIFT_RPM
  TAKEOFF = 2,      -- lift held, then cruise starts and reaches CRUISE_RPM
  LIFT_DOWN = 3,    -- lift ramps to off, cruise drops to CRUISE_LOW_RPM
  CRUISE = 4,       -- cruise alone at CRUISE_LOW_RPM for the hold time
  LANDING = 5,      -- lift ramps up and cruise ramps to off together
  LANDING_HOLD = 6, -- lift alone at LIFT_RPM
  LANDING_DOWN = 7, -- lift ramps to off
}

local TARGET_RPM = {
  [TEST.INDIVIDUAL_LIFT] = 16500,
  [TEST.SHARED_LIFT] = 13000,
  [TEST.CRUISE_SUSTAINED] = 7800,
}

local HOLD_S = {
  [TEST.INDIVIDUAL_LIFT] = 5,
  [TEST.SHARED_LIFT] = 90,
  [TEST.CRUISE_MAX] = 3,
  [TEST.CRUISE_SUSTAINED] = 180,
  [TEST.TRANSITION] = 60,
}

local SHARED_TEMP_MARKS = {
  { s = 45, key = "temp_45s" },
  { s = 60, key = "temp_1m" },
  { s = 90, key = "temp_1m30" },
}

local MAV_SEVERITY = { CRITICAL = 2, WARNING = 4, INFO = 6 }

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), "GT: could not add param table")

local function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format("GT: could not find %s", name))
  return p
end

local function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("GT: could not add %s", name))
  return bind_param(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: GT_TEST
  // @DisplayName: Ground test to run
  // @Description: Motor test run when the Scripting1 switch goes high while armed in MANUAL
  // @Values: 0:None,1:Individual lift motor,2:Shared lift motors,3:Cruise motor max,4:Cruise motor sustained,5:Transition
  // @User: Standard
--]]
local GT_TEST = bind_add_param("TEST", 1, 0)

--[[
  // @Param: GT_MOTOR
  // @DisplayName: Lift motor for the individual test
  // @Description: Lift motor number (Motor1 to Motor4) driven by the individual lift motor test
  // @Values: 1:Front right,2:Back left,3:Front left,4:Back right
  // @User: Standard
--]]
local GT_MOTOR = bind_add_param("MOTOR", 2, 1)

--[[
  // @Param: GT_THR_PCT
  // @DisplayName: Cruise max throttle
  // @Description: Throttle of the cruise motor max test, as a percentage of the DShot range
  // @Range: 0 100
  // @Units: %
  // @User: Standard
--]]
local GT_THR_PCT = bind_add_param("THR_PCT", 5, 80)

--[[
  // @Param: GT_PWM_MAX
  // @DisplayName: Maximum PWM
  // @Description: Hard ceiling on the PWM the script commands to any motor
  // @Range: 1000 2000
  // @Units: PWM
  // @User: Standard
--]]
local GT_PWM_MAX = bind_add_param("PWM_MAX", 6, 1850)

--[[
  // @Param: GT_SLEW
  // @DisplayName: PWM slew rate
  // @Description: Fastest PWM change of the RPM loop and of the cruise motor max ramp. The transition test's timed ramps are not limited by it, and stopping a test always ramps down at 50% of the PWM range per second
  // @Range: 5 500
  // @Units: PWM/s
  // @User: Standard
--]]
local GT_SLEW = bind_add_param("SLEW", 7, 50)

--[[
  // @Param: GT_RPM_TOL
  // @DisplayName: RPM tolerance
  // @Description: The target is reached once the RPM stays this close to it for 2 s
  // @Range: 50 2000
  // @Units: RPM
  // @User: Standard
--]]
local GT_RPM_TOL = bind_add_param("RPM_TOL", 9, 250)

--[[
  // @Param: GT_RPM_OVR
  // @DisplayName: Overspeed margin
  // @Description: Abort when a tested motor runs this far over the target RPM
  // @Range: 2 50
  // @Units: %
  // @User: Standard
--]]
local GT_RPM_OVR = bind_add_param("RPM_OVR", 10, 10)

--[[
  // @Param: GT_ESC_TMAX
  // @DisplayName: ESC temperature limit
  // @Description: Abort when a tested motor's ESC is hotter than this
  // @Range: 30 150
  // @Units: degC
  // @User: Standard
--]]
local GT_ESC_TMAX = bind_add_param("ESC_TMAX", 11, 85)

--[[
  // @Param: GT_CURR_MAX
  // @DisplayName: Battery current limit
  // @Description: Abort when the battery current is above this
  // @Range: 10 400
  // @Units: A
  // @User: Standard
--]]
local GT_CURR_MAX = bind_add_param("CURR_MAX", 12, 150)

--[[
  // @Param: GT_VOLT_MIN
  // @DisplayName: Battery voltage limit
  // @Description: Abort when the battery voltage is below this, 0 to use BATT_LOW_VOLT
  // @Range: 0 100
  // @Units: V
  // @User: Standard
--]]
local GT_VOLT_MIN = bind_add_param("VOLT_MIN", 13, 0)

local RCMAP_THROTTLE = bind_param("RCMAP_THROTTLE")
local BATT_LOW_VOLT = bind_param("BATT_LOW_VOLT")

local switch = rc:find_channel_for_option(AUX_SCRIPTING1)

-- RC channel number of the switch, only to report its PWM
local switch_rc = nil
for i = 1, 16 do
  if param:get(string.format("RC%d_OPTION", i)) == AUX_SCRIPTING1 then
    switch_rc = i
    break
  end
end

local function send(severity, fmt, ...)
  gcs:send_text(severity, "GT: " .. string.format(fmt, ...))
end

local function now_ms()
  return millis():toint()
end

local function test_name(test)
  return SECTION_KEYS[test] or "none"
end

local function motor_channel(func)
  return SRV_Channels:find_channel(func)
end

-- the channel and ESC telemetry index of every motor, nil for one not on an output
local function all_motor_channels()
  local chans = {}
  for _, func in ipairs(LIFT_MOTOR_FUNCTIONS) do
    local chan = motor_channel(func)
    if chan then
      chans[#chans + 1] = chan
    end
  end
  local cruise = motor_channel(CRUISE_MOTOR_FUNCTION)
  if cruise then
    chans[#chans + 1] = cruise
  end
  return chans
end

local function esc_rpm(chan)
  return esc_telem:get_rpm(chan)
end

local function esc_temp_c(chan)
  local cdeg = esc_telem:get_temperature(chan)
  if cdeg == nil then
    return nil
  end
  return cdeg * 0.01
end

local function battery_current()
  return battery:current_amps(0) or 0
end

local function battery_voltage()
  return battery:voltage(0)
end

local function voltage_limit()
  local v = GT_VOLT_MIN:get()
  if v > 0 then
    return v
  end
  return BATT_LOW_VOLT:get()
end

-- throttle stick fraction of its range, nil when the channel cannot be read
local function throttle_stick()
  local ch = math.floor(RCMAP_THROTTLE:get())
  local pwm = rc:get_pwm(ch)
  local min = param:get(string.format("RC%d_MIN", ch))
  local max = param:get(string.format("RC%d_MAX", ch))
  if pwm == nil or min == nil or max == nil or max <= min then
    return nil
  end
  return (pwm - min) / (max - min)
end

-- the outputs a test drives, or nil and the reason it cannot run. The transition
-- test returns the lift motors, its cruise motor is driven separately
local function test_motors(test)
  if test == TEST.INDIVIDUAL_LIFT then
    local motor = math.floor(GT_MOTOR:get())
    local func = LIFT_MOTOR_FUNCTIONS[motor]
    if func == nil then
      return nil, string.format("GT_MOTOR %d is not 1 to 4", motor)
    end
    local chan = motor_channel(func)
    if chan == nil then
      return nil, string.format("Motor%d is not on an output", motor)
    end
    return { chan }
  end
  if test == TEST.SHARED_LIFT or test == TEST.TRANSITION then
    local chans = {}
    for i, func in ipairs(LIFT_MOTOR_FUNCTIONS) do
      local chan = motor_channel(func)
      if chan == nil then
        return nil, string.format("Motor%d is not on an output", i)
      end
      chans[#chans + 1] = chan
    end
    if test == TEST.TRANSITION and motor_channel(CRUISE_MOTOR_FUNCTION) == nil then
      return nil, "the cruise motor (Throttle) is not on an output"
    end
    return chans
  end
  if test == TEST.CRUISE_MAX or test == TEST.CRUISE_SUSTAINED then
    local chan = motor_channel(CRUISE_MOTOR_FUNCTION)
    if chan == nil then
      return nil, "the cruise motor (Throttle) is not on an output"
    end
    return { chan }
  end
  return nil, string.format("GT_TEST %d is not a test", test)
end

local function mean_rpm(chans)
  local sum = 0
  for _, chan in ipairs(chans) do
    local rpm = esc_rpm(chan)
    if rpm == nil then
      return nil
    end
    sum = sum + rpm
  end
  return sum / #chans
end

local function max_temp(chans)
  local hottest = nil
  for _, chan in ipairs(chans) do
    local t = esc_temp_c(chan)
    if t ~= nil and (hottest == nil or t > hottest) then
      hottest = t
    end
  end
  return hottest
end

local function min_temp(chans)
  local coldest = nil
  for _, chan in ipairs(chans) do
    local t = esc_temp_c(chan)
    if t ~= nil and (coldest == nil or t < coldest) then
      coldest = t
    end
  end
  return coldest
end

local function sum_esc_current(chans)
  local sum = 0
  for _, chan in ipairs(chans) do
    sum = sum + (esc_telem:get_current(chan) or 0)
  end
  return sum
end

local STATE = {
  IDLE = 0,
  RAMP = 1,
  HOLD = 2,
  RAMP_DOWN = 3,
  OFF = 4,
}

local state = STATE.IDLE
local run = nil
local switch_was_high = true
local idle = { armed = false, test = nil, switch_pos = nil }
local last_log_ms = 0
local last_named_float_ms = 0

local function log_result(key, value)
  if value == nil then
    return
  end
  logger:write("GTRS", "Tst,Key,Val", "BZf", run.test, SECTION_KEYS[run.test] .. "." .. key, value)
end

local function finish(disarm_reason)
  if arming:is_armed() then
    arming:disarm()
  end
  for _, chan in ipairs(all_motor_channels()) do
    SRV_Channels:set_output_pwm_chan_timeout(chan, PWM_OFF, OVERRIDE_TIMEOUT_MS)
  end
  if disarm_reason then
    send(MAV_SEVERITY.INFO, "disarmed, %s", disarm_reason)
  end
  state = STATE.IDLE
  run = nil
end

-- ramps the motors down before disarming, unless a disarm or motor emergency stop has already stopped them
local function abort(fmt, ...)
  local reason = string.format(fmt, ...)
  logger:write("GTRS", "Tst,Key,Val", "BZf", run.test, string.sub("abort: " .. reason, 1, 63), 0)
  if not arming:is_armed() or SRV_Channels:get_emergency_stop() then
    send(MAV_SEVERITY.CRITICAL, "ABORT %s", reason)
    finish(nil)
    return
  end
  send(MAV_SEVERITY.CRITICAL, "ABORT %s, ramping down", reason)
  run.aborted = true
  state = STATE.RAMP_DOWN
  run.state_start_ms = now_ms()
end

-- reasons that stop a test at once, checked before starting and on every loop
local function hard_stop_reason()
  if switch:get_aux_switch_pos() ~= SWITCH_HIGH then
    return "switch not high"
  end
  if not arming:is_armed() then
    return "disarmed"
  end
  if vehicle:get_mode() ~= MODE_MANUAL then
    return "not in MANUAL"
  end
  if SRV_Channels:get_emergency_stop() then
    return "motor emergency stop"
  end
  if not rc:has_valid_input() then
    return "no RC input"
  end
  local stick = throttle_stick()
  if stick == nil then
    return "cannot read the throttle stick"
  end
  if stick > STICK_LOW_FRACTION then
    return "throttle stick raised"
  end
  return nil
end

local function stale_telem_reason(chans)
  for _, chan in ipairs(chans) do
    if (millis() - esc_telem:get_last_telem_data_ms(chan)):toint() > 500 then
      return string.format("no ESC telemetry on output %d", chan + 1)
    end
  end
  return nil
end

local function start_blocked_reason(test, chans)
  local reason = hard_stop_reason()
  if reason then
    return reason
  end
  reason = stale_telem_reason(chans)
  if reason then
    return reason
  end
  local hottest = max_temp(chans)
  if hottest ~= nil and hottest >= GT_ESC_TMAX:get() then
    return string.format("ESC at %.0fC already", hottest)
  end
  local volts = battery_voltage()
  if volts < voltage_limit() then
    return string.format("battery %.1fV below %.1fV", volts, voltage_limit())
  end
  return nil
end

local function start(test)
  local chans, reason = test_motors(test)
  if chans == nil then
    send(MAV_SEVERITY.WARNING, "cannot start, %s", reason)
    return
  end
  -- every motor the test drives, which the telemetry and limit checks cover
  local tested = {}
  for i, chan in ipairs(chans) do
    tested[i] = chan
  end
  local cruise = nil
  if test == TEST.TRANSITION then
    local cruise_chan = motor_channel(CRUISE_MOTOR_FUNCTION)
    -- see group below for the fields
    cruise = { chans = { cruise_chan }, pwm = PWM_OFF, mode = "off", target_rpm = nil, cmd_ms = 0 }
    tested[#tested + 1] = cruise_chan
  end
  reason = start_blocked_reason(test, tested)
  if reason then
    send(MAV_SEVERITY.WARNING, "cannot start, %s", reason)
    return
  end

  local t = now_ms()
  run = {
    test = test,
    chans = chans,
    tested = tested,
    cruise = cruise,
    phase = cruise and TR_PHASE.LIFT_UP or nil,
    phase_start_ms = t,
    tmax_lift = nil,
    tmax_cruise = nil,
    others = {},
    pwm = PWM_OFF,
    target_rpm = TARGET_RPM[test],
    target_pwm = nil,
    state_start_ms = t,
    last_ms = t,
    in_band_since = nil,
    persist = {},
    hold = { n = 0, pwm = 0, rpm = 0, bcur = 0, ecur = 0, tmax = nil },
    marks_done = 0,
  }
  if test == TEST.CRUISE_MAX then
    run.target_pwm = PWM_OFF + (PWM_FULL - PWM_OFF) * GT_THR_PCT:get() * 0.01
    run.target_rpm = nil
  end
  if test == TEST.TRANSITION then
    -- the lift starts in the RPM loop, the overspeed check waits for it to reach the target
    run.lift = { chans = chans, pwm = PWM_OFF, mode = "rpm", loop_rpm = TR.LIFT_RPM, target_rpm = nil, cmd_ms = t }
    run.target_rpm = nil
  end
  for _, chan in ipairs(all_motor_channels()) do
    local is_tested = false
    for _, c in ipairs(tested) do
      if c == chan then
        is_tested = true
      end
    end
    if not is_tested then
      run.others[#run.others + 1] = chan
    end
  end

  if test == TEST.SHARED_LIFT then
    local lo = min_temp(chans)
    local hi = max_temp(chans)
    if lo ~= nil and hi ~= nil then
      log_result("starting_esc_temp_range.min", lo)
      log_result("starting_esc_temp_range.max", hi)
      send(MAV_SEVERITY.INFO, "starting ESC temps %.0f-%.0fC", lo, hi)
    end
    log_result("battery_starting_voltage.result", battery_voltage())
  end

  state = STATE.RAMP
  if run.lift then
    send(MAV_SEVERITY.INFO, "%s started, lift to %.0f RPM", test_name(test), TR.LIFT_RPM)
  elseif run.target_rpm then
    send(MAV_SEVERITY.INFO, "%s started, target %.0f RPM", test_name(test), run.target_rpm)
  else
    send(MAV_SEVERITY.INFO, "%s started, target %.0f PWM", test_name(test), run.target_pwm)
  end
end

-- true once cond has held for persist_ms, so a single noisy sample never aborts
local function persisted(name, cond, persist_ms, t)
  if not cond then
    run.persist[name] = nil
    return false
  end
  if run.persist[name] == nil then
    run.persist[name] = t
  end
  return t - run.persist[name] >= persist_ms
end

-- overspeed and stall checks on one group of motors driven on one PWM
local function motor_limit_reason(chans, pwm, target, t)
  for _, chan in ipairs(chans) do
    local rpm = esc_rpm(chan) or 0
    if target then
      local ceiling = target * (1 + GT_RPM_OVR:get() * 0.01)
      if persisted("overspeed" .. chan, rpm > ceiling, 300, t) then
        return string.format("output %d at %.0f RPM, over %.0f", chan + 1, rpm, ceiling)
      end
    end
    if persisted("stall" .. chan, pwm > STALL_PWM and rpm < STALL_RPM, 1000, t) then
      return string.format("output %d not spinning at %.0f PWM", chan + 1, pwm)
    end
  end
  return nil
end

local function limit_reason(t)
  local reason = stale_telem_reason(run.tested)
  if reason then
    return reason
  end
  local hottest = max_temp(run.tested)
  if persisted("temp", hottest ~= nil and hottest > GT_ESC_TMAX:get(), LIMIT_PERSIST_MS, t) then
    return string.format("ESC %.1fC over %.0fC", hottest, GT_ESC_TMAX:get())
  end
  local amps = battery_current()
  if persisted("current", amps > GT_CURR_MAX:get(), LIMIT_PERSIST_MS, t) then
    return string.format("battery %.0fA over %.0fA", amps, GT_CURR_MAX:get())
  end
  local volts = battery_voltage()
  if persisted("voltage", volts < voltage_limit(), LIMIT_PERSIST_MS, t) then
    return string.format("battery %.1fV below %.1fV", volts, voltage_limit())
  end
  reason = motor_limit_reason(run.chans, run.pwm, run.target_rpm, t)
  if reason == nil and run.cruise then
    reason = motor_limit_reason(run.cruise.chans, run.cruise.pwm, run.cruise.target_rpm, t)
  end
  return reason
end

local function slew_toward(pwm, goal, rate, dt)
  local step = rate * dt
  if goal > pwm then
    return math.min(goal, pwm + step)
  end
  return math.max(goal, pwm - step)
end

-- integrates the RPM error into PWM, limited to the slew rate and the PWM ceiling,
-- returns the new PWM and the mean RPM
local function rpm_loop(chans, pwm, target, dt)
  local rpm = mean_rpm(chans) or 0
  local slew = GT_SLEW:get()
  -- 0.02 PWM per second for each RPM of error
  local rate = 0.02 * (target - rpm)
  rate = math.max(-slew, math.min(slew, rate))
  return math.max(PWM_OFF, math.min(GT_PWM_MAX:get(), pwm + rate * dt)), rpm
end

local function drive(dt)
  if run.target_rpm then
    local rpm
    run.pwm, rpm = rpm_loop(run.chans, run.pwm, run.target_rpm, dt)
    return rpm
  end
  run.pwm = slew_toward(run.pwm, math.min(run.target_pwm, GT_PWM_MAX:get()), GT_SLEW:get(), dt)
  return mean_rpm(run.chans) or 0
end

-- true once rpm has stayed within GT_RPM_TOL of target for 2 s
local function settled(rpm, target, t)
  if math.abs(rpm - target) > GT_RPM_TOL:get() then
    run.in_band_since = nil
    return false
  end
  if run.in_band_since == nil then
    run.in_band_since = t
  end
  return t - run.in_band_since >= 2000
end

local function enter(new_state, t)
  state = new_state
  run.state_start_ms = t
end

local function accumulate_hold()
  local h = run.hold
  h.n = h.n + 1
  h.pwm = h.pwm + run.pwm
  h.rpm = h.rpm + (mean_rpm(run.chans) or 0)
  h.bcur = h.bcur + battery_current()
  h.ecur = h.ecur + sum_esc_current(run.chans)
  local hottest = max_temp(run.chans)
  if hottest ~= nil and (h.tmax == nil or hottest > h.tmax) then
    h.tmax = hottest
  end
end

local function shared_temp_marks(t)
  local elapsed = t - run.state_start_ms
  local mark = SHARED_TEMP_MARKS[run.marks_done + 1]
  if mark and elapsed >= mark.s * 1000 then
    local hottest = max_temp(run.chans)
    log_result(mark.key .. ".result", hottest)
    send(MAV_SEVERITY.INFO, "%ds hottest lift ESC %.0fC", mark.s, hottest or -1)
    run.marks_done = run.marks_done + 1
  end
end

local function report_hold()
  local h = run.hold
  if h.n == 0 then
    return
  end
  local pwm = h.pwm / h.n
  local rpm = h.rpm / h.n
  local bcur = h.bcur / h.n
  local ecur = h.ecur / h.n
  local test = run.test
  if test == TEST.INDIVIDUAL_LIFT then
    local row = LIFT_ROW_KEYS[math.floor(GT_MOTOR:get())]
    log_result(row .. ".current_a", bcur)
    log_result(row .. ".esc_current_a", ecur)
    log_result(row .. ".rcout", pwm)
    send(MAV_SEVERITY.INFO, "%s %.0f RPM, %.1fA batt, %.1fA ESC, RCOUT %.0f", row, rpm, bcur, ecur, pwm)
  elseif test == TEST.SHARED_LIFT then
    log_result("current_13000_rpm.result", bcur)
    log_result("rcout_13000_rpm.result", pwm)
    send(MAV_SEVERITY.INFO, "lift %.0f RPM, %.1fA, RCOUT %.0f", rpm, bcur, pwm)
  elseif test == TEST.CRUISE_MAX then
    log_result("rpm_100t.result", rpm)
    log_result("current_100.result", bcur)
    log_result("rc_out.result", pwm)
    send(MAV_SEVERITY.INFO, "cruise %.0f RPM, %.1fA, RC OUT %.0f", rpm, bcur, pwm)
  elseif test == TEST.CRUISE_SUSTAINED then
    log_result("current_a.result", bcur)
    log_result("rc_out.result", pwm)
    log_result("esc_max_temp.result", h.tmax)
    send(MAV_SEVERITY.INFO, "cruise %.0f RPM, %.1fA, RC OUT %.0f, ESC max %.0fC", rpm, bcur, pwm, h.tmax or -1)
  end
end

local function enter_phase(phase, t, fmt, ...)
  run.phase = phase
  run.phase_start_ms = t
  send(MAV_SEVERITY.INFO, fmt, ...)
end

--[[
  a group of motors on one PWM in the transition test (run.lift, run.cruise), in one of
  these modes:
    off   held at PWM_OFF
    rpm   closed loop to loop_rpm, hit_ms is when it first came within GT_RPM_TOL
    ramp  PWM moves linearly from ramp_from to ramp_to over ramp_ms
  target_rpm is the overspeed target, set once the RPM loop has reached loop_rpm so a
  group coming down to a lower target is not over it
--]]
local group = {}

function group.rpm(g, rpm, t)
  g.mode = "rpm"
  g.loop_rpm = rpm
  g.hit_ms = nil
  g.target_rpm = nil
  g.cmd_ms = t
end

function group.ramp(g, to_pwm, ms, t)
  g.mode = "ramp"
  g.ramp_from = g.pwm
  g.ramp_to = math.min(to_pwm, GT_PWM_MAX:get())
  g.ramp_ms = ms
  g.target_rpm = nil
  g.cmd_ms = t
end

function group.ramp_done(g, t)
  return g.mode == "ramp" and t - g.cmd_ms >= g.ramp_ms
end

-- steps the group's PWM and returns its mean RPM
function group.update(g, t, dt)
  if g.mode == "rpm" then
    local rpm
    g.pwm, rpm = rpm_loop(g.chans, g.pwm, g.loop_rpm, dt)
    if g.hit_ms == nil and math.abs(rpm - g.loop_rpm) <= GT_RPM_TOL:get() then
      g.hit_ms = t
      g.target_rpm = g.loop_rpm
    end
    return rpm
  end
  if g.mode == "ramp" then
    local frac = math.min(1, (t - g.cmd_ms) / g.ramp_ms)
    g.pwm = g.ramp_from + (g.ramp_to - g.ramp_from) * frac
  else
    g.pwm = PWM_OFF
  end
  return mean_rpm(g.chans) or 0
end

-- the transition test, with the lift motors and the cruise motor each on their own PWM
local function step_transition(t, dt)
  local lift = run.lift
  local cruise = run.cruise
  local phase = run.phase
  local in_phase = t - run.phase_start_ms

  -- the results are the hottest lift and cruise ESCs over the whole test
  local lift_hot = max_temp(lift.chans)
  if lift_hot ~= nil and (run.tmax_lift == nil or lift_hot > run.tmax_lift) then
    run.tmax_lift = lift_hot
  end
  local cruise_hot = max_temp(cruise.chans)
  if cruise_hot ~= nil and (run.tmax_cruise == nil or cruise_hot > run.tmax_cruise) then
    run.tmax_cruise = cruise_hot
  end

  local lift_rpm = group.update(lift, t, dt)
  local cruise_rpm = group.update(cruise, t, dt)
  -- the generic limit checks, outputs, ramp down and log use run.pwm and run.target_rpm for the lift
  run.pwm = lift.pwm
  run.target_rpm = lift.target_rpm

  if lift.mode == "rpm" and lift.hit_ms == nil and t - lift.cmd_ms > RAMP_TIMEOUT_MS then
    abort("lift not at %.0f RPM in %.0fs", lift.loop_rpm, RAMP_TIMEOUT_MS / 1000)
    return
  end
  if cruise.mode == "rpm" and cruise.hit_ms == nil and t - cruise.cmd_ms > RAMP_TIMEOUT_MS then
    abort("cruise not at %.0f RPM in %.0fs", cruise.loop_rpm, RAMP_TIMEOUT_MS / 1000)
    return
  end

  if phase == TR_PHASE.LIFT_UP then
    if lift.hit_ms then
      enter_phase(TR_PHASE.TAKEOFF, t, "lift at %.0f RPM, holding %.0fs", lift_rpm, TR.LIFT_HOLD_MS / 1000)
    end
  elseif phase == TR_PHASE.TAKEOFF then
    if cruise.mode == "off" and in_phase >= TR.LIFT_HOLD_MS then
      group.rpm(cruise, TR.CRUISE_RPM, t)
      send(MAV_SEVERITY.INFO, "cruise up to %.0f RPM", TR.CRUISE_RPM)
    end
    if cruise.hit_ms and run.cruise_high_ms == nil then
      run.cruise_high_ms = cruise.hit_ms
      send(MAV_SEVERITY.INFO, "cruise at %.0f RPM", cruise_rpm)
    end
    if run.cruise_high_ms and t - run.cruise_high_ms >= TR.CRUISE_LEAD_MS then
      run.lift_hold_pwm = lift.pwm
      group.ramp(lift, PWM_OFF, TR.LIFT_DOWN_MS, t)
      enter_phase(TR_PHASE.LIFT_DOWN, t, "lift down over %.0fs from %.0f PWM", TR.LIFT_DOWN_MS / 1000, lift.pwm)
    end
  elseif phase == TR_PHASE.LIFT_DOWN then
    if cruise.loop_rpm == TR.CRUISE_RPM and t - run.cruise_high_ms >= TR.CRUISE_HIGH_MS then
      group.rpm(cruise, TR.CRUISE_LOW_RPM, t)
      send(MAV_SEVERITY.INFO, "cruise down to %.0f RPM", TR.CRUISE_LOW_RPM)
    end
    if group.ramp_done(lift, t) and cruise.loop_rpm == TR.CRUISE_LOW_RPM and cruise.hit_ms then
      enter_phase(TR_PHASE.CRUISE, t, "lift off, cruise alone at %.0f RPM for %.0fs", cruise_rpm, HOLD_S[run.test])
    end
  elseif phase == TR_PHASE.CRUISE then
    -- the mode is checked every loop, so leaving MANUAL has already aborted
    if in_phase >= HOLD_S[run.test] * 1000 then
      group.ramp(lift, run.lift_hold_pwm, TR.LANDING_RAMP_MS, t)
      group.ramp(cruise, PWM_OFF, TR.LANDING_RAMP_MS, t)
      enter_phase(TR_PHASE.LANDING, t, "still in MANUAL, lift up and cruise down over %.0fs", TR.LANDING_RAMP_MS / 1000)
    end
  elseif phase == TR_PHASE.LANDING then
    if group.ramp_done(lift, t) then
      group.rpm(lift, TR.LIFT_RPM, t)
      enter_phase(TR_PHASE.LANDING_HOLD, t, "cruise off, lift to %.0f RPM", TR.LIFT_RPM)
    end
  elseif phase == TR_PHASE.LANDING_HOLD then
    -- the landing hold times from the lift reaching LIFT_RPM
    if lift.hit_ms and run.lift_hit_sent == nil then
      run.lift_hit_sent = true
      send(MAV_SEVERITY.INFO, "lift at %.0f RPM, holding %.0fs", lift_rpm, TR.LANDING_HOLD_MS / 1000)
    end
    if lift.hit_ms and t - lift.hit_ms >= TR.LANDING_HOLD_MS then
      group.ramp(lift, PWM_OFF, TR.LANDING_DOWN_MS, t)
      enter_phase(TR_PHASE.LANDING_DOWN, t, "lift down over %.0fs", TR.LANDING_DOWN_MS / 1000)
    end
  elseif phase == TR_PHASE.LANDING_DOWN then
    if group.ramp_done(lift, t) then
      log_result("lift_esc_max_temp.result", run.tmax_lift)
      log_result("cruise_esc_max_temp.result", run.tmax_cruise)
      send(MAV_SEVERITY.INFO, "lift off, lift ESC max %.0fC, cruise ESC max %.0fC",
        run.tmax_lift or -1, run.tmax_cruise or -1)
      enter(STATE.OFF, t)
    end
  end
end

local function step_states(t, dt)
  if run.cruise and (state == STATE.RAMP or state == STATE.HOLD) then
    step_transition(t, dt)
  elseif state == STATE.RAMP then
    local rpm = drive(dt)
    local reached
    if run.target_rpm then
      reached = settled(rpm, run.target_rpm, t)
    else
      reached = run.pwm >= math.min(run.target_pwm, GT_PWM_MAX:get())
    end
    if reached then
      send(MAV_SEVERITY.INFO, "target reached at %.0f PWM, holding %.0fs", run.pwm, HOLD_S[run.test])
      enter(STATE.HOLD, t)
    elseif t - run.state_start_ms > RAMP_TIMEOUT_MS then
      abort("target not reached in %.0fs at %.0f PWM", RAMP_TIMEOUT_MS / 1000, run.pwm)
    end
  elseif state == STATE.HOLD then
    drive(dt)
    accumulate_hold()
    if run.test == TEST.SHARED_LIFT then
      shared_temp_marks(t)
    end
    if t - run.state_start_ms >= HOLD_S[run.test] * 1000 then
      report_hold()
      enter(STATE.RAMP_DOWN, t)
    end
  elseif state == STATE.RAMP_DOWN then
    run.pwm = slew_toward(run.pwm, PWM_OFF, RAMP_DOWN_PWM_S, dt)
    if run.cruise then
      run.cruise.pwm = slew_toward(run.cruise.pwm, PWM_OFF, RAMP_DOWN_PWM_S, dt)
    end
    if run.pwm <= PWM_OFF and (run.cruise == nil or run.cruise.pwm <= PWM_OFF) then
      enter(STATE.OFF, t)
    end
  elseif state == STATE.OFF then
    if t - run.state_start_ms >= 1000 then
      if run.aborted then
        finish("ramped down after abort")
      else
        send(MAV_SEVERITY.INFO, "%s complete", test_name(run.test))
        finish("test complete")
      end
    end
  end
end

local function write_outputs()
  local pwm = math.floor(run.pwm + 0.5)
  for _, chan in ipairs(run.chans) do
    SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, OVERRIDE_TIMEOUT_MS)
  end
  if run.cruise then
    local cruise_pwm = math.floor(run.cruise.pwm + 0.5)
    for _, chan in ipairs(run.cruise.chans) do
      SRV_Channels:set_output_pwm_chan_timeout(chan, cruise_pwm, OVERRIDE_TIMEOUT_MS)
    end
  end
  for _, chan in ipairs(run.others) do
    SRV_Channels:set_output_pwm_chan_timeout(chan, PWM_OFF, OVERRIDE_TIMEOUT_MS)
  end
end

local function log_status(t)
  if t - last_log_ms >= 100 then
    last_log_ms = t
    local cruise = run.cruise
    logger:write("GTST", "Tst,St,Ph,Tgt,PWM,RPM,CPWM,CRPM,BCur,ECur,Volt,ETmp", "BBBfffffffff",
      run.test, state, run.phase or 0, run.target_rpm or run.target_pwm or 0, run.pwm, mean_rpm(run.chans) or 0,
      cruise and cruise.pwm or 0, cruise and mean_rpm(cruise.chans) or 0,
      battery_current(), sum_esc_current(run.tested), battery_voltage(), max_temp(run.tested) or 0)
  end
  if t - last_named_float_ms >= 200 then
    last_named_float_ms = t
    gcs:send_named_float("GT_RPM", mean_rpm(run.chans) or 0)
    gcs:send_named_float("GT_PWM", run.pwm)
    if run.cruise then
      gcs:send_named_float("GT_CRPM", mean_rpm(run.cruise.chans) or 0)
      gcs:send_named_float("GT_CPWM", run.cruise.pwm)
    end
  end
end

local function update_running(t)
  local dt = (t - run.last_ms) * 0.001
  run.last_ms = t

  local reason
  if state == STATE.RAMP_DOWN or state == STATE.OFF then
    -- already stopping, only a disarm or motor emergency stop ends the ramp down early
    if not arming:is_armed() then
      reason = "disarmed"
    elseif SRV_Channels:get_emergency_stop() then
      reason = "motor emergency stop"
    end
  else
    reason = hard_stop_reason()
    if reason == nil and math.floor(GT_TEST:get()) ~= run.test then
      reason = "GT_TEST changed"
    end
    if reason == nil then
      reason = limit_reason(t)
    end
  end
  if reason then
    abort("%s", reason)
    return
  end

  step_states(t, dt)
  if run then
    write_outputs()
    log_status(t)
  end
end

-- statustext on idle changes, so the operator can see why a test has not started
local function report_idle(test, switch_pos, armed)
  if test ~= idle.test then
    if test == 0 then
      send(MAV_SEVERITY.INFO, "no test selected, set GT_TEST")
    else
      send(MAV_SEVERITY.INFO, "GT_TEST %d selected, %s", test, test_name(test))
    end
  end
  if armed and not idle.armed then
    if switch_pos == SWITCH_HIGH then
      send(MAV_SEVERITY.INFO, "armed with switch high, move it low then high to start")
    else
      send(MAV_SEVERITY.INFO, "armed, switch high to start %s", test_name(test))
    end
  end
  if switch_pos ~= idle.switch_pos then
    local name = ({ [0] = "low", [1] = "middle", [2] = "high" })[switch_pos] or tostring(switch_pos)
    local pwm = switch_rc and rc:get_pwm(switch_rc)
    if pwm then
      send(MAV_SEVERITY.INFO, "switch %s, RC%d at %d PWM, high is over 1800", name, switch_rc, pwm)
    else
      send(MAV_SEVERITY.INFO, "switch %s", name)
    end
  end
  idle.test = test
  idle.armed = armed
  idle.switch_pos = switch_pos
end

local function update()
  local t = now_ms()
  local switch_pos = switch:get_aux_switch_pos()
  local switch_high = switch_pos == SWITCH_HIGH

  if state == STATE.IDLE then
    local test = math.floor(GT_TEST:get())
    report_idle(test, switch_pos, arming:is_armed())
    -- a test only starts on the switch moving to high, never on arming with it already high
    if switch_high and not switch_was_high then
      if test == 0 then
        send(MAV_SEVERITY.WARNING, "switch high, but no test selected, set GT_TEST")
      else
        send(MAV_SEVERITY.INFO, "switch high, starting %s", test_name(test))
        start(test)
      end
    end
  else
    update_running(t)
  end

  switch_was_high = switch_high
end

local function protected_update()
  local ok, err = pcall(update)
  if ok then
    return protected_update, 20
  end
  gcs:send_text(MAV_SEVERITY.CRITICAL, "GT: script error " .. tostring(err))
  if state ~= STATE.IDLE then
    finish(nil)
  end
  switch_was_high = true
  return protected_update, 1000
end

if switch == nil then
  gcs:send_text(MAV_SEVERITY.CRITICAL, "GT: set an RCx_OPTION to 300 (Scripting1) for the run switch")
  return
end

send(MAV_SEVERITY.INFO, "ground test loaded, run switch RC%d", switch_rc or 0)
return protected_update, 1000
