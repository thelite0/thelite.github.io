local args = {...}
local TEST_MODE = args[1] == "test"
local QUIET_TEST = args[2] == "quiet"

local RANGE = 2048
local SIREN_SIDE = "top"
local SCAN_INTERVAL = 1.0
local HISTORY = 8
local TRACK_TTL = 12

-- Known rival-sector direction in NeoRadar local coordinates (radar - target).
local SECTOR_X, SECTOR_Z = 1881, 1632
local SECTOR_LEN = math.sqrt(SECTOR_X * SECTOR_X + SECTOR_Z * SECTOR_Z)
local SECTOR_COS = 0.70

local gpu, gpuSide
for _, name in ipairs(peripheral.getNames()) do
  local p = peripheral.wrap(name)
  if p and type(p.refreshSize) == "function" and type(p.sync) == "function" and type(p.getSize) == "function" then
    gpu, gpuSide = p, name
    break
  end
end
assert(gpu, "compatible display controller not found")

-- Tom's GPU refresh is asynchronous. Wait until a bitmap display is discovered.
local W, H = 0, 0
for _ = 1, 30 do
  gpu.refreshSize()
  sleep(0.1)
  W, H = gpu.getSize()
  if type(W) == "number" and type(H) == "number" and W > 0 and H > 0 then break end
end
assert(W > 0 and H > 0, "no bitmap display detected by GPU")

gpu.setSize(64)
sleep(0.1)
W, H = gpu.getSize()

local radar
for _, name in ipairs(peripheral.getNames()) do
  local p = peripheral.wrap(name)
  if p and type(p.scanForSubLevels) == "function" then
    radar = p
    break
  end
end
if not TEST_MODE then assert(radar, "neo radar not found") end

local C = {
  bg      = 0xFF050A0D,
  panel   = 0xFF0A1217,
  panel2  = 0xFF0D1A20,
  grid    = 0xFF173039,
  border  = 0xFF27505D,
  text    = 0xFFE3F0F3,
  dim     = 0xFF70838A,
  cyan    = 0xFF49D9F2,
  green   = 0xFF4FE39B,
  yellow  = 0xFFF0D15A,
  orange  = 0xFFF39A52,
  red     = 0xFFFF5D67,
  magenta = 0xFFFF65D4,
  white   = 0xFFFFFFFF,
}

local MIN_X, MIN_Y = 2, 2
local MAX_X, MAX_Y = math.max(2, W - 2), math.max(2, H - 2)

local function clamp(v, a, b)
  if v < a then return a end
  if v > b then return b end
  return v
end

local function rect(x, y, w, h, c)
  x, y, w, h = math.floor(x), math.floor(y), math.floor(w), math.floor(h)
  if w < 1 or h < 1 then return end
  x = clamp(x, MIN_X, MAX_X)
  y = clamp(y, MIN_Y, MAX_Y)
  w = math.min(w, MAX_X - x)
  h = math.min(h, MAX_Y - y)
  if w >= 1 and h >= 1 then gpu.filledRectangle(x, y, w, h, c) end
end

local function outline(x, y, w, h, c)
  x, y, w, h = math.floor(x), math.floor(y), math.floor(w), math.floor(h)
  x = clamp(x, MIN_X, MAX_X)
  y = clamp(y, MIN_Y, MAX_Y)
  w = math.min(w, MAX_X - x)
  h = math.min(h, MAX_Y - y)
  if w >= 2 and h >= 2 then gpu.rectangle(x, y, w, h, c) end
end

local function line(x1, y1, x2, y2, c)
  x1 = clamp(math.floor(x1), MIN_X, MAX_X)
  y1 = clamp(math.floor(y1), MIN_Y, MAX_Y)
  x2 = clamp(math.floor(x2), MIN_X, MAX_X)
  y2 = clamp(math.floor(y2), MIN_Y, MAX_Y)
  gpu.lineS(x1, y1, x2, y2, c)
end

local function txt(x, y, s, c, size)
  x = clamp(math.floor(x), MIN_X, MAX_X - 2)
  y = clamp(math.floor(y), MIN_Y, MAX_Y - 8)
  pcall(gpu.drawText, x, y, tostring(s), c or C.text, 0x00000000, size or 1)
end

local function shortNum(v)
  if not v then return "--" end
  if math.abs(v) >= 1000 then return string.format("%.1fk", v / 1000) end
  return string.format("%.0f", v)
end

local function keyOf(v)
  local q = v.id
  if q then
    return tostring(q[1]) .. ":" .. tostring(q[2]) .. ":" .. tostring(q[3]) .. ":" .. tostring(q[4])
  end
  return tostring(v.name or "contact") .. ":" .. tostring(math.floor(v.x or 0)) .. ":" .. tostring(math.floor(v.z or 0))
end

local function linearSlope(h, field)
  local n = #h
  if n < 2 then return 0 end
  local mt, mv = 0, 0
  for i = 1, n do
    mt = mt + h[i].t
    mv = mv + h[i][field]
  end
  mt, mv = mt / n, mv / n
  local num, den = 0, 0
  for i = 1, n do
    local dt = h[i].t - mt
    num = num + dt * (h[i][field] - mv)
    den = den + dt * dt
  end
  if den <= 0 then return 0 end
  return num / den
end

local function threatFor(d, closing, samples, sector)
  local eta = closing > 1 and d / closing or nil
  local label, rank = "TRACK", 0
  if sector then label, rank = "WATCH", 1 end

  local threshold = d > 1500 and 12 or 6
  local inbound = sector and samples >= 4 and closing > threshold
  if inbound then
    label, rank = "INBOUND", 2
    if d < 1200 or (eta and eta < 60) then label, rank = "HIGH", 3 end
    if d < 500 or (eta and eta < 20) then label, rank = "CRIT", 4 end
  end
  return label, rank, eta, inbound
end

local tracks = {}
local rows = {}
local scanOK = true
local scanError = nil
local lastScan = 0
local lastScanMs = 0
local sweep = 0
local testStart = os.epoch("utc") / 1000

local function scanLive(now)
  local started = os.epoch("utc")
  local ok, contacts = pcall(radar.scanForSubLevels, RANGE)
  lastScanMs = os.epoch("utc") - started
  scanOK = ok
  scanError = ok and nil or tostring(contacts)
  if not ok then return end

  local seen = {}
  local out = {}

  for _, v in ipairs(contacts) do
    local k = keyOf(v)
    seen[k] = true
    local tr = tracks[k] or {history = {}, first = now, last = now}
    tracks[k] = tr
    tr.last = now
    tr.name = v.name or tr.name

    local h = tr.history
    h[#h + 1] = {t = now, d = v.distance or 0, x = v.x or 0, y = v.y or 0, z = v.z or 0}
    while #h > HISTORY do table.remove(h, 1) end

    local dd = linearSlope(h, "d")
    local vx = linearSlope(h, "x")
    local vy = linearSlope(h, "y")
    local vz = linearSlope(h, "z")
    local closing = -dd
    local speed = math.sqrt(vx * vx + vy * vy + vz * vz)

    local hz = math.sqrt((v.x or 0)^2 + (v.z or 0)^2)
    local sector = hz > 0 and (((v.x or 0) * SECTOR_X + (v.z or 0) * SECTOR_Z) / (hz * SECTOR_LEN) > SECTOR_COS)
    local label, rank, eta, inbound = threatFor(v.distance or 0, closing, #h, sector)

    out[#out + 1] = {
      key = k,
      name = v.name or "UNKNOWN",
      x = v.x or 0,
      y = v.y or 0,
      z = v.z or 0,
      d = v.distance or 0,
      closing = closing,
      speed = speed,
      eta = eta,
      sector = sector,
      label = label,
      rank = rank,
      inbound = inbound,
      samples = #h,
    }
  end

  for k, tr in pairs(tracks) do
    if not seen[k] and now - tr.last > TRACK_TTL then tracks[k] = nil end
  end

  table.sort(out, function(a, b)
    if a.rank ~= b.rank then return a.rank > b.rank end
    return a.d < b.d
  end)
  rows = out
end

local function scanTest(now)
  scanOK = true
  scanError = nil
  lastScanMs = 3

  local phase = (now - testStart) % 24
  local rank, label
  if phase < 3 then rank, label = 1, "WATCH"
  elseif phase < 8 then rank, label = 2, "INBOUND"
  elseif phase < 14 then rank, label = 3, "HIGH"
  elseif phase < 19 then rank, label = 4, "CRIT"
  else rank, label = 0, "TRACK" end

  local progress = clamp(phase / 19, 0, 1)
  local d = 2050 - progress * 1750
  local ratioX, ratioZ = SECTOR_X / SECTOR_LEN, SECTOR_Z / SECTOR_LEN
  local closing = rank >= 2 and 38 or (rank == 1 and 8 or -4)
  local eta = closing > 1 and d / closing or nil

  rows = {
    {
      key = "SIM:ALPHA",
      name = "SIM-01",
      x = ratioX * d,
      y = 80,
      z = ratioZ * d,
      d = d,
      closing = closing,
      speed = math.abs(closing) + 2.4,
      eta = eta,
      sector = true,
      label = label,
      rank = rank,
      inbound = rank >= 2,
      samples = 8,
    },
    {
      key = "SIM:BRAVO",
      name = "SIM-02",
      x = -760,
      y = -20,
      z = 420,
      d = 870,
      closing = -2.5,
      speed = 4.1,
      eta = nil,
      sector = false,
      label = "TRACK",
      rank = 0,
      inbound = false,
      samples = 8,
    }
  }
end

local function colorFor(rank)
  if rank >= 4 then return C.magenta end
  if rank == 3 then return C.red end
  if rank == 2 then return C.orange end
  if rank == 1 then return C.yellow end
  return C.green
end

local function drawCross(cx, cy, c)
  line(cx - 2, cy, cx + 2, cy, c)
  line(cx, cy - 2, cx, cy + 2, c)
end

local function drawUI(now)
  gpu.fill(C.bg)

  local top = rows[1]
  local rank = top and top.rank or 0
  local status = top and top.label or "CLEAR"
  local statusColor = colorFor(rank)

  -- Header.
  rect(3, 3, W - 7, 9, C.panel2)
  rect(3, 11, W - 7, 1, statusColor)
  txt(6, 4, "EARLY WARNING", C.text, 1)
  if TEST_MODE then
    txt(W - 26, 4, "TEST", C.cyan, 1)
  else
    txt(W - 30, 4, status, statusColor, 1)
  end

  -- Scope: square because NeoRadar actually scans an AABB, not a sphere.
  local sx, sy = 3, 15
  local side = math.min(42, H - 19, W - 20)
  side = math.max(24, side)
  rect(sx, sy, side, side, C.panel)
  outline(sx, sy, side, side, C.border)

  local cx = sx + math.floor(side / 2)
  local cy = sy + math.floor(side / 2)
  local half = math.floor(side / 2) - 3

  -- Grid and range rings.
  line(cx, sy + 2, cx, sy + side - 2, C.grid)
  line(sx + 2, cy, sx + side - 2, cy, C.grid)
  outline(cx - math.floor(half * .5), cy - math.floor(half * .5), math.floor(half), math.floor(half), C.grid)

  -- Rival-sector bearing marker (towards target in world-relative coordinates).
  local worldDX, worldDZ = -SECTOR_X / SECTOR_LEN, -SECTOR_Z / SECTOR_LEN
  line(cx, cy, cx + worldDX * half, cy + worldDZ * half, 0xFF203D46)

  -- Sweep.
  local sweepLen = half
  line(cx, cy, cx + math.cos(sweep) * sweepLen, cy + math.sin(sweep) * sweepLen, C.cyan)

  drawCross(cx, cy, C.cyan)
  txt(sx + 3, sy + 2, "N", C.dim, 1)

  -- Contacts. NeoRadar gives radar-target, so invert for world-relative map plotting.
  for _, c in ipairs(rows) do
    local relX, relZ = -c.x, -c.z
    local px = cx + clamp(relX / RANGE, -1, 1) * half
    local py = cy + clamp(relZ / RANGE, -1, 1) * half
    local col = colorFor(c.rank)
    if c.rank >= 3 and math.floor(now * 5) % 2 == 0 then col = C.white end
    rect(px - 1, py - 1, 3, 3, col)
    if c.rank >= 2 then outline(px - 3, py - 3, 7, 7, col) end
  end

  -- Right telemetry rail.
  local rx = sx + side + 3
  local rw = W - rx - 3
  if rw >= 12 then
    rect(rx, sy, rw, side, C.panel)
    outline(rx, sy, rw, side, C.border)

    txt(rx + 3, sy + 3, status, statusColor, 1)
    txt(rx + 3, sy + 12, "C " .. tostring(#rows), C.dim, 1)

    if top then
      txt(rx + 3, sy + 21, "D " .. shortNum(top.d), C.text, 1)
      txt(rx + 3, sy + 30, "V " .. string.format("%+.0f", top.closing), top.closing > 5 and C.orange or C.text, 1)
      local etaText = top.eta and (string.format("%.0fs", top.eta)) or "--"
      txt(rx + 3, sy + 39, "T " .. etaText, C.text, 1)
    end
  end

  -- Footer / health strip.
  local fy = math.min(H - 10, sy + side + 3)
  rect(3, fy, W - 7, 7, C.panel2)
  local radarCol = scanOK and C.green or C.red
  txt(6, fy + 1, scanOK and "SCAN OK" or "SCAN ERR", radarCol, 1)
  txt(34, fy + 1, "R" .. tostring(RANGE), C.dim, 1)

  local alarm = false
  for _, c in ipairs(rows) do if c.inbound then alarm = true break end end
  if TEST_MODE and QUIET_TEST then alarm = false end
  txt(W - 22, fy + 1, alarm and "SIREN" or "ARMED", alarm and C.red or C.green, 1)

  gpu.sync()
  return alarm
end

local function shutdown()
  pcall(redstone.setOutput, SIREN_SIDE, false)
  pcall(gpu.fill, C.bg)
  pcall(gpu.sync)
end

local ok, err = pcall(function()
  while true do
    local now = os.epoch("utc") / 1000

    if now - lastScan >= SCAN_INTERVAL then
      lastScan = now
      if TEST_MODE then scanTest(now) else scanLive(now) end
    end

    local alarm = drawUI(now)
    redstone.setOutput(SIREN_SIDE, alarm)

    sweep = (sweep + 0.08) % (math.pi * 2)
    sleep(0.08)
  end
end)

shutdown()
if not ok then error(err, 0) end
