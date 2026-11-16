--[[
  Marble wing heater duty cycling

  The heater switch reads one servo output: above 1900 is on, below 1100 is
  off. Set that output's SERVOn_FUNCTION to the scripting output chosen by
  WHT_SCR_OUT (94 for Script1) and its SERVOn_TRIM to 1000.

  WHT_MODE 1 cycles the heater WHT_ON_S on and WHT_OFF_S off, WHT_MODE 0 keeps
  it off. Every parameter is read each loop, so a parameter write from the
  ground takes effect at once. The heater runs armed or disarmed.

  The on command is an override that lapses after 300 ms, after which the
  output falls back to SERVOn_TRIM, so if the script stops for any reason the
  heater turns off.
--]]

local PARAM_TABLE_KEY = 62
local PARAM_TABLE_PREFIX = "WHT_"

local LOOP_MS = 100
local OVERRIDE_TIMEOUT_MS = 300
local REPORT_INTERVAL_MS = 1000
local SCRIPTING1_FUNCTION = 94

local MODE_OFF = 0
local MODE_CYCLE = 1

local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO = 6

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), "WHT: could not add param table")

local function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("WHT: could not add %s", name))
  local p = Parameter()
  assert(p:init(PARAM_TABLE_PREFIX .. name), string.format("WHT: could not find %s", name))
  return p
end

--[[
  // @Param: WHT_MODE
  // @DisplayName: Wing heater mode
  // @Description: Off, or cycling on for WHT_ON_S and off for WHT_OFF_S
  // @Values: 0:Off,1:Cycle
  // @User: Standard
--]]
local WHT_MODE = bind_add_param("MODE", 1, 0)

--[[
  // @Param: WHT_ON_S
  // @DisplayName: Wing heater on time
  // @Description: Time the heater is on in each cycle, 0 keeps it off
  // @Range: 0 600
  // @Units: s
  // @User: Standard
--]]
local WHT_ON_S = bind_add_param("ON_S", 2, 10)

--[[
  // @Param: WHT_OFF_S
  // @DisplayName: Wing heater off time
  // @Description: Time the heater is off in each cycle
  // @Range: 0 600
  // @Units: s
  // @User: Standard
--]]
local WHT_OFF_S = bind_add_param("OFF_S", 3, 20)

--[[
  // @Param: WHT_SCR_OUT
  // @DisplayName: Wing heater scripting output
  // @Description: Scripting output the heater switch is on, 1 for Script1 (SERVOn_FUNCTION 94)
  // @Range: 1 16
  // @User: Advanced
--]]
local WHT_SCR_OUT = bind_add_param("SCR_OUT", 4, 1)

--[[
  // @Param: WHT_PWM_ON
  // @DisplayName: Wing heater on PWM
  // @Description: PWM sent while the heater is on, above the switch's 1900 threshold
  // @Range: 1901 2200
  // @Units: PWM
  // @User: Advanced
--]]
local WHT_PWM_ON = bind_add_param("PWM_ON", 5, 2000)

local function send(severity, fmt, ...)
  gcs:send_text(severity, "WHT: " .. string.format(fmt, ...))
end

local function now_ms()
  return millis():toint()
end

local function output_function()
  return SCRIPTING1_FUNCTION + math.floor(WHT_SCR_OUT:get()) - 1
end

local function on_ms()
  return math.max(0, WHT_ON_S:get()) * 1000
end

local function off_ms()
  return math.max(0, WHT_OFF_S:get()) * 1000
end

local heater_on = false
local phase_start_ms = 0
local last_mode = MODE_OFF
local last_report_ms = 0
local cleared_functions = {}

-- a scripting output starts with a stored PWM, so the override would hold the
-- last on value when it lapsed; setting its scaled output once clears that and
-- leaves the lapsed output at SERVOn_TRIM
local function prepare_output(func)
  if not cleared_functions[func] then
    SRV_Channels:set_output_scaled(func, 0)
    cleared_functions[func] = true
  end
end

local function command_on(func)
  local chan = SRV_Channels:find_channel(func)
  if chan == nil then
    return false
  end
  SRV_Channels:set_output_pwm_chan_timeout(chan, math.floor(WHT_PWM_ON:get()), OVERRIDE_TIMEOUT_MS)
  return true
end

local function set_heater(on, t)
  if on ~= heater_on then
    heater_on = on
    phase_start_ms = t
  end
end

local function report(t)
  if t - last_report_ms < REPORT_INTERVAL_MS then
    return
  end
  last_report_ms = t
  local mode = math.floor(WHT_MODE:get())
  local state = heater_on and 1 or 0
  gcs:send_named_float("WHT_MODE", mode)
  gcs:send_named_float("WHT_ON", state)
  logger:write("WHT", "Mode,On", "BB", mode, state)
end

local function update()
  local t = now_ms()
  local mode = math.floor(WHT_MODE:get())
  local func = output_function()
  prepare_output(func)

  if mode ~= last_mode then
    if mode == MODE_CYCLE then
      send(MAV_SEVERITY_INFO, "cycling %.0fs on, %.0fs off", on_ms() / 1000, off_ms() / 1000)
      heater_on = false
      set_heater(true, t)
    else
      send(MAV_SEVERITY_INFO, "off")
    end
    last_mode = mode
  end

  if mode ~= MODE_CYCLE then
    set_heater(false, t)
  elseif heater_on and t - phase_start_ms >= on_ms() then
    set_heater(false, t)
  elseif not heater_on and t - phase_start_ms >= off_ms() then
    set_heater(on_ms() > 0, t)
  end

  if heater_on and not command_on(func) then
    send(MAV_SEVERITY_WARNING, "no output has SERVOn_FUNCTION %d, setting WHT_MODE 0", func)
    WHT_MODE:set_and_save(MODE_OFF)
    set_heater(false, t)
    last_mode = MODE_OFF
  end

  report(t)
end

local function protected_update()
  local ok, err = pcall(update)
  if ok then
    return protected_update, LOOP_MS
  end
  -- not commanding on lets the override lapse, which turns the heater off
  heater_on = false
  gcs:send_text(MAV_SEVERITY_WARNING, "WHT: script error " .. tostring(err))
  return protected_update, 1000
end

WHT_MODE:set_and_save(MODE_OFF)
send(MAV_SEVERITY_INFO, "wing heater loaded, off")
return protected_update, 1000
