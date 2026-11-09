--[[
  LangitSwarm LED player for ArduPilot (Lua scripting)            TEMPLATE
  ------------------------------------------------------------------------
  Plays the drone's exported light file (lights/drone_XXXX.lights from the
  LangitSwarm show package) on NeoPixel / ProfiLED LEDs, synchronised to GPS time.

  !! Bench-test and SITL-test before any flight. API names follow ArduPilot 4.5+.

  Setup on every drone
    SCR_ENABLE = 1                        (reboot, then copy this file to APM/scripts/)
    copy the drone's light file to       APM/scripts/show.lights
    SERVOx_FUNCTION = 94 (Script1)        on the output wired to the LED data line
    NUM_LEDS below = number of LEDs

  Synchronisation
    LangitSwarm sets SCR_USER1 = show start time as seconds of the UTC day just
    before take-off.  Show time = GPS UTC time - SCR_USER1.  GPS week seconds are
    used directly: the GPS epoch starts at midnight UTC and a week is a whole
    number of days, so  seconds_of_day = (time_of_week - leap_seconds) mod 86400.
    If SCR_USER1 is 0 the player falls back to time since arming.

  File format: 20-byte header ("LSL1", uint16 fps, uint16 version, uint32 frames,
  uint32 drone index, uint32 reserved, little endian), then 3 bytes RGB per frame.
--]]

local FILE = "APM/scripts/show.lights"
local NUM_LEDS = 1
local LED_FUNCTION = 94          -- Script1
local GPS_LEAP_SECONDS = 18      -- GPS - UTC (since 2017)
local UPDATE_MS = 40             -- 25 Hz refresh

local chan = SRV_Channels:find_channel(LED_FUNCTION)
if not chan then
  gcs:send_text(3, "LSW lights: no output with SERVOx_FUNCTION=94")
  return
end
chan = chan + 1
serialLED:set_num_neopixel(chan, NUM_LEDS)

local f = io.open(FILE, "rb")
if not f then
  gcs:send_text(3, "LSW lights: " .. FILE .. " missing")
  return
end
local hdr = f:read(20)
if not hdr or #hdr < 20 or hdr:sub(1, 4) ~= "LSL1" then
  gcs:send_text(3, "LSW lights: not a LangitSwarm light file")
  return
end
local fps, _, frames, drone = string.unpack("<I2I2I4I4", hdr, 5)
gcs:send_text(6, string.format("LSW lights: drone %d, %d frames @ %d fps", drone + 1, frames, fps))

local arm_ms = nil
local last_frame = -2

local function show_rgb(r, g, b)
  serialLED:set_RGB(chan, -1, r, g, b)
  serialLED:send(chan)
end

local function utc_seconds_of_day()
  if gps:status(0) < 3 then
    return nil
  end
  local tow_ms = gps:time_week_ms(0)                -- uint32 userdata
  local whole = (tow_ms / 1000):toint()             -- whole seconds of the GPS week
  local frac = (tow_ms % 1000):toint() / 1000.0
  return ((whole - GPS_LEAP_SECONDS) % 86400) + frac
end

local function show_time()
  local start = param:get("SCR_USER1") or 0
  if start > 0 then
    local sod = utc_seconds_of_day()
    if sod then
      local t = sod - start
      if t < -43200 then t = t + 86400 elseif t > 43200 then t = t - 86400 end
      return t
    end
  end
  if arm_ms then
    return (millis():tofloat() - arm_ms) / 1000.0
  end
  return nil
end

local function update()
  if arming:is_armed() then
    if not arm_ms then arm_ms = millis():tofloat() end
  else
    arm_ms = nil
  end
  local t = show_time()
  local k = -1
  if t and t >= 0 then
    k = math.floor(t * fps)
    if k >= frames then k = -1 end
  end
  if k ~= last_frame then
    last_frame = k
    if k < 0 then
      show_rgb(0, 0, 0)
    else
      f:seek("set", 20 + 3 * k)
      local px = f:read(3)
      if px and #px == 3 then
        show_rgb(px:byte(1), px:byte(2), px:byte(3))
      end
    end
  end
  return update, UPDATE_MS
end

show_rgb(0, 0, 0)
return update, 1000
