--[[
  Marble ground test automation for the QuadPlane motor tests

  Runs one motor test of the Control ground test at a time, chosen with GT_TEST:
    1 individual lift motor (GT_MOTOR), closed loop on ESC RPM
    2 shared lift motors, all four on one PWM, closed loop on their mean RPM
    3 cruise motor max, open loop to GT_THR_PCT
    4 cruise motor sustained, closed loop on ESC RPM

  The operator arms in MANUAL with the throttle stick low, then moves the
  Scripting1 switch (RCx_OPTION 300) to high to start. Anything but high
  aborts. The script drives the motor outputs directly by overriding their
  PWM with a short timeout, so no SERVOn_FUNCTION changes are needed and the
  outputs fall back on their own if the script stops. Every abort, and the end
  of every test, disarms, which makes the DShot outputs send zero.

  Results are logged as GTRS messages keyed by the Control ground test field
  key, and the run is logged at 10 Hz as GTST.
--]]

local PARAM_TABLE_KEY = 61
local PARAM_TABLE_PREFIX = "GT_"

local LOOP_MS = 20
local OVERRIDE_TIMEOUT_MS = 60
local LOG_INTERVAL_MS = 100
local NAMED_FLOAT_INTERVAL_MS = 200

local MODE_MANUAL = 0
local AUX_SCRIPTING1 = 300
local SWITCH_HIGH = 2

local PWM_OFF = 1000
local PWM_FULL = 2000
local LIFT_MOTOR_FUNCTIONS = { 33, 34, 35, 36 }
local CRUISE_MOTOR_FUNCTION = 70
local LIFT_ROW_KEYS = { "front_right", "back_left", "front_left", "back_right" }

local TELEM_STALE_MS = 500
local STICK_LOW_FRACTION = 0.1
local LIMIT_PERSIST_MS = 500
local OVERSPEED_PERSIST_MS = 300
local STALL_PWM = 1150
local STALL_RPM = 300
local STALL_PERSIST_MS = 1000
local RAMP_TIMEOUT_MS = 60000
local SETTLE_MS = 2000
local RAMP_DOWN_SLEW_FACTOR = 2
local OFF_HOLD_MS = 1000

local TEST_INDIVIDUAL_LIFT = 1
local TEST_SHARED_LIFT = 2
local TEST_CRUISE_MAX = 3
local TEST_CRUISE_SUSTAINED = 4

local SECTION_KEYS = {
  [TEST_INDIVIDUAL_LIFT] = "individual_lift_motor",
  [TEST_SHARED_LIFT] = "shared_lift_motor",
  [TEST_CRUISE_MAX] = "cruise_motor_max",
  [TEST_CRUISE_SUSTAINED] = "cruise_motor_sustained",
}

local DEFAULT_RPM = {
  [TEST_INDIVIDUAL_LIFT] = 16500,
  [TEST_SHARED_LIFT] = 13000,
  [TEST_CRUISE_SUSTAINED] = 7800,
}

local DEFAULT_HOLD_S = {
  [TEST_INDIVIDUAL_LIFT] = 10,
  [TEST_SHARED_LIFT] = 90,
  [TEST_CRUISE_MAX] = 3,
  [TEST_CRUISE_SUSTAINED] = 180,
}

local SHARED_TEMP_MARKS = {
  { s = 45, key = "temp_45s" },
  { s = 60, key = "temp_1m" },
  { s = 90, key = "temp_1m30" },
}

local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO = 6

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
  // @Values: 0:None,1:Individual lift motor,2:Shared lift motors,3:Cruise motor max,4:Cruise motor sustained
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
  // @Param: GT_RPM
  // @DisplayName: Target RPM
  // @Description: Target RPM of a closed loop test, 0 for the test's default (16500 individual lift, 13000 shared lift, 7800 cruise sustained)
  // @Range: 0 30000
  // @Units: RPM
  // @User: Standard
--]]
local GT_RPM = bind_add_param("RPM", 3, 0)

--[[
  // @Param: GT_HOLD
  // @DisplayName: Hold time
  // @Description: Time held at the target once reached, 0 for the test's default (10 s individual lift, 90 s shared lift, 3 s cruise max, 180 s cruise sustained)
  // @Range: 0 600
  // @Units: s
  // @User: Standard
--]]
local GT_HOLD = bind_add_param("HOLD", 4, 0)

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
  // @Description: Fastest PWM change while ramping up, ramping down runs at twice this
  // @Range: 5 500
  // @Units: PWM/s
  // @User: Standard
--]]
local GT_SLEW = bind_add_param("SLEW", 7, 50)

--[[
  // @Param: GT_RPM_KI
  // @DisplayName: RPM loop gain
  // @Description: PWM change per second for each RPM of error in the closed loop tests, limited by GT_SLEW
  // @Range: 0.001 0.2
  // @User: Advanced
--]]
local GT_RPM_KI = bind_add_param("RPM_KI", 8, 0.02)

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

local function esc_current(chan)
  return esc_telem:get_current(chan)
end

local function esc_telem_age_ms(chan)
  return (millis() - esc_telem:get_last_telem_data_ms(chan)):toint()
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

local function target_rpm(test)
  local rpm = GT_RPM:get()
  if rpm > 0 then
    return rpm
  end
  return DEFAULT_RPM[test]
end

local function hold_ms(test)
  local s = GT_HOLD:get()
  if s <= 0 then
    s = DEFAULT_HOLD_S[test]
  end
  if test == TEST_SHARED_LIFT then
    s = math.max(s, SHARED_TEMP_MARKS[#SHARED_TEMP_MARKS].s)
  end
  return s * 1000
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

-- the outputs a test drives, or nil and the reason it cannot run
local function test_motors(test)
  if test == TEST_INDIVIDUAL_LIFT then
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
  if test == TEST_SHARED_LIFT then
    local chans = {}
    for i, func in ipairs(LIFT_MOTOR_FUNCTIONS) do
      local chan = motor_channel(func)
      if chan == nil then
        return nil, string.format("Motor%d is not on an output", i)
      end
      chans[#chans + 1] = chan
    end
    return chans
  end
  if test == TEST_CRUISE_MAX or test == TEST_CRUISE_SUSTAINED then
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
    sum = sum + (esc_current(chan) or 0)
  end
  return sum
end

local STATE_IDLE = 0
local STATE_RAMP = 1
local STATE_HOLD = 2
local STATE_RAMP_DOWN = 3
local STATE_OFF = 4

local state = STATE_IDLE
local run = nil
local switch_was_high = true
local last_log_ms = 0
local last_named_float_ms = 0

local function log_result(key, value)
  if value == nil then
    return
  end
  logger:write("GTRS", "Tst,Key,Val", "BZf", run.test, SECTION_KEYS[run.test] .. "." .. key, value)
end

local function release_outputs()
  for _, chan in ipairs(all_motor_channels()) do
    SRV_Channels:set_output_pwm_chan_timeout(chan, PWM_OFF, OVERRIDE_TIMEOUT_MS)
  end
end

local function finish(disarm_reason)
  if arming:is_armed() then
    arming:disarm()
  end
  release_outputs()
  if disarm_reason then
    send(MAV_SEVERITY_INFO, "disarmed, %s", disarm_reason)
  end
  state = STATE_IDLE
  run = nil
end

local function abort(fmt, ...)
  local reason = string.format(fmt, ...)
  if run then
    logger:write("GTRS", "Tst,Key,Val", "BZf", run.test, string.sub("abort: " .. reason, 1, 63), 0)
  end
  send(MAV_SEVERITY_CRITICAL, "ABORT %s", reason)
  finish(nil)
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
    if esc_telem_age_ms(chan) > TELEM_STALE_MS then
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
  if test ~= TEST_CRUISE_MAX and target_rpm(test) == nil then
    return "no target RPM"
  end
  return nil
end

local function start(test)
  local chans, reason = test_motors(test)
  if chans == nil then
    send(MAV_SEVERITY_WARNING, "cannot start, %s", reason)
    return
  end
  reason = start_blocked_reason(test, chans)
  if reason then
    send(MAV_SEVERITY_WARNING, "cannot start, %s", reason)
    return
  end

  local t = now_ms()
  run = {
    test = test,
    chans = chans,
    others = {},
    pwm = PWM_OFF,
    target_rpm = target_rpm(test),
    target_pwm = nil,
    state_start_ms = t,
    last_ms = t,
    in_band_since = nil,
    persist = {},
    hold = { n = 0, pwm = 0, rpm = 0, bcur = 0, ecur = 0, tmax = nil },
    marks_done = 0,
  }
  if test == TEST_CRUISE_MAX then
    run.target_pwm = PWM_OFF + (PWM_FULL - PWM_OFF) * GT_THR_PCT:get() * 0.01
    run.target_rpm = nil
  end
  for _, chan in ipairs(all_motor_channels()) do
    local tested = false
    for _, c in ipairs(chans) do
      if c == chan then
        tested = true
      end
    end
    if not tested then
      run.others[#run.others + 1] = chan
    end
  end

  if test == TEST_SHARED_LIFT then
    local lo = min_temp(chans)
    local hi = max_temp(chans)
    if lo ~= nil and hi ~= nil then
      log_result("starting_esc_temp_range.min", lo)
      log_result("starting_esc_temp_range.max", hi)
      send(MAV_SEVERITY_INFO, "starting ESC temps %.0f-%.0fC", lo, hi)
    end
    log_result("battery_starting_voltage.result", battery_voltage())
  end

  state = STATE_RAMP
  if run.target_rpm then
    send(MAV_SEVERITY_INFO, "%s started, target %.0f RPM", test_name(test), run.target_rpm)
  else
    send(MAV_SEVERITY_INFO, "%s started, target %.0f PWM", test_name(test), run.target_pwm)
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

local function limit_reason(t)
  local reason = stale_telem_reason(run.chans)
  if reason then
    return reason
  end
  local hottest = max_temp(run.chans)
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
  for _, chan in ipairs(run.chans) do
    local rpm = esc_rpm(chan) or 0
    if run.target_rpm then
      local ceiling = run.target_rpm * (1 + GT_RPM_OVR:get() * 0.01)
      if persisted("overspeed" .. chan, rpm > ceiling, OVERSPEED_PERSIST_MS, t) then
        return string.format("output %d at %.0f RPM, over %.0f", chan + 1, rpm, ceiling)
      end
    end
    if persisted("stall" .. chan, run.pwm > STALL_PWM and rpm < STALL_RPM, STALL_PERSIST_MS, t) then
      return string.format("output %d not spinning at %.0f PWM", chan + 1, run.pwm)
    end
  end
  return nil
end

local function slew_toward(goal, rate, dt)
  local step = rate * dt
  if goal > run.pwm then
    run.pwm = math.min(goal, run.pwm + step)
  else
    run.pwm = math.max(goal, run.pwm - step)
  end
end

-- integrates the RPM error into PWM, limited to the slew rate and the PWM ceiling
local function rpm_loop(dt)
  local rpm = mean_rpm(run.chans) or 0
  local slew = GT_SLEW:get()
  local rate = GT_RPM_KI:get() * (run.target_rpm - rpm)
  rate = math.max(-slew, math.min(slew, rate))
  run.pwm = math.max(PWM_OFF, math.min(GT_PWM_MAX:get(), run.pwm + rate * dt))
  return rpm
end

local function drive(dt)
  if run.target_rpm then
    return rpm_loop(dt)
  end
  slew_toward(math.min(run.target_pwm, GT_PWM_MAX:get()), GT_SLEW:get(), dt)
  return mean_rpm(run.chans) or 0
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
    send(MAV_SEVERITY_INFO, "%ds hottest lift ESC %.0fC", mark.s, hottest or -1)
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
  if test == TEST_INDIVIDUAL_LIFT then
    local row = LIFT_ROW_KEYS[math.floor(GT_MOTOR:get())]
    log_result(row .. ".current_a", bcur)
    log_result(row .. ".esc_current_a", ecur)
    log_result(row .. ".rcout", pwm)
    send(MAV_SEVERITY_INFO, "%s %.0f RPM, %.1fA batt, %.1fA ESC, RCOUT %.0f", row, rpm, bcur, ecur, pwm)
  elseif test == TEST_SHARED_LIFT then
    log_result("current_13000_rpm.result", bcur)
    log_result("rcout_13000_rpm.result", pwm)
    send(MAV_SEVERITY_INFO, "lift %.0f RPM, %.1fA, RCOUT %.0f", rpm, bcur, pwm)
  elseif test == TEST_CRUISE_MAX then
    log_result("rpm_100t.result", rpm)
    log_result("current_100.result", bcur)
    log_result("rc_out.result", pwm)
    send(MAV_SEVERITY_INFO, "cruise %.0f RPM, %.1fA, RC OUT %.0f", rpm, bcur, pwm)
  elseif test == TEST_CRUISE_SUSTAINED then
    log_result("current_a.result", bcur)
    log_result("rc_out.result", pwm)
    log_result("esc_max_temp.result", h.tmax)
    send(MAV_SEVERITY_INFO, "cruise %.0f RPM, %.1fA, RC OUT %.0f, ESC max %.0fC", rpm, bcur, pwm, h.tmax or -1)
  end
end

local function step_states(t, dt)
  if state == STATE_RAMP then
    local rpm = drive(dt)
    local reached
    if run.target_rpm then
      local in_band = math.abs(rpm - run.target_rpm) <= GT_RPM_TOL:get()
      if not in_band then
        run.in_band_since = nil
      elseif run.in_band_since == nil then
        run.in_band_since = t
      end
      reached = run.in_band_since ~= nil and t - run.in_band_since >= SETTLE_MS
    else
      reached = run.pwm >= math.min(run.target_pwm, GT_PWM_MAX:get())
    end
    if reached then
      send(MAV_SEVERITY_INFO, "target reached at %.0f PWM, holding %.0fs", run.pwm, hold_ms(run.test) / 1000)
      enter(STATE_HOLD, t)
    elseif t - run.state_start_ms > RAMP_TIMEOUT_MS then
      abort("target not reached in %.0fs at %.0f PWM", RAMP_TIMEOUT_MS / 1000, run.pwm)
    end
  elseif state == STATE_HOLD then
    drive(dt)
    accumulate_hold()
    if run.test == TEST_SHARED_LIFT then
      shared_temp_marks(t)
    end
    if t - run.state_start_ms >= hold_ms(run.test) then
      report_hold()
      enter(STATE_RAMP_DOWN, t)
    end
  elseif state == STATE_RAMP_DOWN then
    slew_toward(PWM_OFF, GT_SLEW:get() * RAMP_DOWN_SLEW_FACTOR, dt)
    if run.pwm <= PWM_OFF then
      enter(STATE_OFF, t)
    end
  elseif state == STATE_OFF then
    if t - run.state_start_ms >= OFF_HOLD_MS then
      send(MAV_SEVERITY_INFO, "%s complete", test_name(run.test))
      finish("test complete")
    end
  end
end

local function write_outputs()
  local pwm = math.floor(run.pwm + 0.5)
  for _, chan in ipairs(run.chans) do
    SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, OVERRIDE_TIMEOUT_MS)
  end
  for _, chan in ipairs(run.others) do
    SRV_Channels:set_output_pwm_chan_timeout(chan, PWM_OFF, OVERRIDE_TIMEOUT_MS)
  end
end

local function log_status(t)
  if t - last_log_ms >= LOG_INTERVAL_MS then
    last_log_ms = t
    logger:write("GTST", "Tst,St,Tgt,PWM,RPM,BCur,ECur,Volt,ETmp", "BBfffffff",
      run.test, state, run.target_rpm or run.target_pwm, run.pwm, mean_rpm(run.chans) or 0,
      battery_current(), sum_esc_current(run.chans), battery_voltage(), max_temp(run.chans) or 0)
  end
  if t - last_named_float_ms >= NAMED_FLOAT_INTERVAL_MS then
    last_named_float_ms = t
    gcs:send_named_float("GT_RPM", mean_rpm(run.chans) or 0)
    gcs:send_named_float("GT_PWM", run.pwm)
  end
end

local function update_running(t)
  local dt = (t - run.last_ms) * 0.001
  run.last_ms = t

  local reason = hard_stop_reason()
  if reason then
    abort("%s", reason)
    return
  end
  if math.floor(GT_TEST:get()) ~= run.test then
    abort("GT_TEST changed")
    return
  end
  reason = limit_reason(t)
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

local function update()
  local t = now_ms()
  local switch_high = switch:get_aux_switch_pos() == SWITCH_HIGH

  if state == STATE_IDLE then
    local test = math.floor(GT_TEST:get())
    -- a test only starts on the switch moving to high, never on arming with it already high
    if switch_high and not switch_was_high and test ~= 0 then
      start(test)
    end
  else
    update_running(t)
  end

  switch_was_high = switch_high
end

local function protected_update()
  local ok, err = pcall(update)
  if ok then
    return protected_update, LOOP_MS
  end
  gcs:send_text(MAV_SEVERITY_CRITICAL, "GT: script error " .. tostring(err))
  if state ~= STATE_IDLE then
    finish(nil)
  end
  switch_was_high = true
  return protected_update, 1000
end

if switch == nil then
  gcs:send_text(MAV_SEVERITY_CRITICAL, "GT: set an RCx_OPTION to 300 (Scripting1) for the run switch")
  return
end

send(MAV_SEVERITY_INFO, "ground test loaded")
return protected_update, 1000
