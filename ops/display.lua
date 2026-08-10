local args = {...}
local TEST_MODE = args[1] == "test"
local QUIET_TEST = args[2] == "quiet"

local RANGE = 2048
local SIREN_SIDE = "back"
local SCAN_INTERVAL = 1.0
local HISTORY = 8
local TRACK_TTL = 12

-- Monitored direction in local sensor coordinates.
local SECTOR_X, SECTOR_Z = 1881, 1632
local SECTOR_LEN = math.sqrt(SECTOR_X * SECTOR_X + SECTOR_Z * SECTOR_Z)
local SECTOR_COS = 0.70

local function nowSeconds()
  return os.epoch("utc") / 1000
end

-- Find the display controller regardless of attachment side.
local gpu, gpuSide
for _, name in ipairs(peripheral.getNames()) do
  local p = peripheral.wrap(name)
  if p and type(p.refreshSize) == "function" and type(p.sync) == "function" and type(p.getSize) == "function" then
    gpu, gpuSide = p, name
    break
  end
end
assert(gpu, "compatible display controller not found")

-- Display size refresh is asynchronous.
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
if not TEST_MODE then assert(radar, "sensor not found") end

local C = {
  bg       = 0xFF05090C,
  panel    = 0xFF0B1216,
  panel2   = 0xFF101B20,
  border   = 0xFF31515A,
  grid     = 0xFF183138,
  text     = 0xFFE5EFF2,
  dim      = 0xFF73868D,
  cyan     = 0xFF49D9F2,
  green    = 0xFF4FE39B,
  yellow   = 0xFFF0D15A,
  orange   = 0xFFFF9D3A,
  red      = 0xFFFF5C68,
  magenta  = 0xFFFF69CF,
  clearBg  = 0xFF10291E,
  watchBg  = 0xFF312A12,
  alertBg  = 0xFF35161A,
  hazardA  = 0xFFF0C94B,
  hazardB  = 0xFF171717,
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
  w = math.min(w, MAX_X - x + 1)
  h = math.min(h, MAX_Y - y + 1)
  if w >= 1 and h >= 1 then gpu.filledRectangle(x, y, w, h, c) end
end

local function outline(x, y, w, h, c)
  x, y, w, h = math.floor(x), math.floor(y), math.floor(w), math.floor(h)
  x = clamp(x, MIN_X, MAX_X)
  y = clamp(y, MIN_Y, MAX_Y)
  w = math.min(w, MAX_X - x + 1)
  h = math.min(h, MAX_Y - y + 1)
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

local function textWidth(s, size)
  return #tostring(s) * 6 * (size or 1)
end

local function txtCentered(y, s, c, size, x, w)
  x = x or 0
  w = w or W
  local px = x + math.floor((w - textWidth(s, size)) / 2)
  txt(px, y, s, c, size)
end

-- Rectangular warning rail. Unlike diagonal primitives, this stays inside its panel.
local function hazardRail(x, y, w, h)
  rect(x, y, w, h, C.hazardB)
  local cell = 8
  local i = 0
  while i * cell < w do
    local px = x + i * cell
    local ww = math.min(cell, w - i * cell)
    if i % 2 == 0 then rect(px, y, ww, h, C.hazardA) end
    i = i + 1
  end
end

local function short(s, n)
  s = tostring(s or "UNKNOWN")
  if #s <= n then return s end
  return s:sub(1, math.max(1, n - 3)) .. "..."
end

local function keyOf(v)
  if type(v.id) == "table" then
    local parts = {}
    for _, k in ipairs(v.id) do parts[#parts + 1] = tostring(k) end
    if #parts > 0 then return table.concat(parts, ":") end
  elseif v.id ~= nil then
    return tostring(v.id)
  end
  return tostring(v.name or "CONTACT") .. "@" .. math.floor(v.x or 0) .. ":" .. math.floor(v.z or 0)
end

local function inSector(x, z)
  local h = math.sqrt(x * x + z * z)
  if h < 1 then return false end
  return (x * SECTOR_X + z * SECTOR_Z) / (h * SECTOR_LEN) >= SECTOR_COS
end

local tracks = {}

local function ensureTrack(key)
  if not tracks[key] then tracks[key] = {history = {}} end
  return tracks[key]
end

local function addSample(track, sample, now)
  local h = track.history
  h[#h + 1] = {
    t = now,
    d = sample.distance or 0,
    x = sample.x or 0,
    y = sample.y or 0,
    z = sample.z or 0,
  }
  while #h > HISTORY do table.remove(h, 1) end
  track.lastSeen = now
end

-- Regression smooths sensor jitter better than a one-sample delta.
local function closingRate(h)
  if #h < 4 then return 0 end
  local t0 = h[1].t
  local n, st, sd, stt, std = #h, 0, 0, 0, 0
  for _, p in ipairs(h) do
    local t = p.t - t0
    st = st + t
    sd = sd + p.d
    stt = stt + t * t
    std = std + t * p.d
  end
  local den = n * stt - st * st
  if math.abs(den) < 0.0001 then return 0 end
  return -((n * std - st * sd) / den)
end

local function totalSpeed(h)
  if #h < 2 then return 0 end
  local a, b = h[1], h[#h]
  local dt = b.t - a.t
  if dt <= 0 then return 0 end
  local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
  return math.sqrt(dx * dx + dy * dy + dz * dz) / dt
end

local function classify(row, sampleCount)
  row.rank = 0
  row.state = "CONTACT"

  if row.sector then
    row.rank = 1
    row.state = "WATCH"

    local threshold = row.d > 1500 and 10 or row.d > 900 and 6 or 3
    if sampleCount >= 4 and row.closing > threshold then
      row.rank = 2
      row.state = "INBOUND"
      if row.d < 1200 or (row.eta and row.eta < 70) then
        row.rank = 3
        row.state = "HIGH"
      end
      if row.d < 550 or (row.eta and row.eta < 25) then
        row.rank = 4
        row.state = "CRIT"
      end
    end
  end
end

local function scanReal(now)
  local raw = radar.scanForSubLevels(RANGE) or {}
  local seen = {}
  local rows = {}

  for _, v in ipairs(raw) do
    v.distance = v.distance or math.sqrt((v.x or 0)^2 + (v.y or 0)^2 + (v.z or 0)^2)
    local key = keyOf(v)
    seen[key] = true
    local tr = ensureTrack(key)
    addSample(tr, v, now)

    local closing = closingRate(tr.history)
    local row = {
      key = key,
      name = v.name or "UNKNOWN",
      x = v.x or 0,
      y = v.y or 0,
      z = v.z or 0,
      d = v.distance,
      closing = closing,
      speed = totalSpeed(tr.history),
      eta = closing > 1 and (v.distance / closing) or nil,
      sector = inSector(v.x or 0, v.z or 0),
    }
    classify(row, #tr.history)
    rows[#rows + 1] = row
  end

  for key, tr in pairs(tracks) do
    if not seen[key] and tr.lastSeen and now - tr.lastSeen > TRACK_TTL then
      tracks[key] = nil
    end
  end

  table.sort(rows, function(a, b)
    if a.rank ~= b.rank then return a.rank > b.rank end
    return a.d < b.d
  end)
  return rows
end

-- Full test cycle: CLEAR -> CONTACT -> WATCH -> INBOUND -> HIGH -> CRIT -> CLEAR.
local function scanTest(now)
  local phase = now % 48
  if phase < 6 or phase >= 42 then return {} end

  if phase < 12 then
    return {{
      key = "TEST-SCOUT", name = "SCOUT", x = -900, y = 20, z = -700,
      d = 1140, closing = 0, speed = 0, eta = nil,
      sector = false, rank = 0, state = "CONTACT",
    }}
  end

  if phase < 18 then
    return {{
      key = "TEST-WATCH", name = "AIRCRAFT", x = 1700, y = 30, z = 1450,
      d = 2235, closing = 0, speed = 0, eta = nil,
      sector = true, rank = 1, state = "WATCH",
    }}
  end

  local rank, state, d, closing
  if phase < 28 then
    rank, state = 2, "INBOUND"
    d = 1900 - (phase - 18) * 70
    closing = 70
  elseif phase < 36 then
    rank, state = 3, "HIGH"
    d = 1150 - (phase - 28) * 85
    closing = 85
  else
    rank, state = 4, "CRIT"
    d = math.max(180, 470 - (phase - 36) * 70)
    closing = 70
  end

  local dx = SECTOR_X / SECTOR_LEN
  local dz = SECTOR_Z / SECTOR_LEN
  return {{
    key = "TEST-THREAT", name = "AIRCRAFT",
    x = dx * d, y = 25, z = dz * d,
    d = d, closing = closing, speed = closing + 3,
    eta = d / closing, sector = true, rank = rank, state = state,
  }}
end

local function overall(rows)
  if #rows == 0 then
    return "CLEAR", "NO CONTACTS DETECTED", C.green, C.clearBg, false
  end

  local top = rows[1]
  if top.rank == 0 then
    local n = #rows
    return "CONTACT", tostring(n) .. (n == 1 and " CONTACT DETECTED" or " CONTACTS DETECTED"), C.cyan, C.panel2, false
  elseif top.rank == 1 then
    return "WATCH", "CONTACT IN WATCH SECTOR", C.yellow, C.watchBg, false
  elseif top.rank == 2 then
    return "INBOUND", "APPROACHING CONTACT", C.orange, C.alertBg, true
  elseif top.rank == 3 then
    return "HIGH", "THREAT - SIREN ACTIVE", C.red, C.alertBg, true
  else
    return "CRITICAL", "IMMEDIATE THREAT", C.magenta, C.alertBg, true
  end
end

local function drawCoverage(x, y, w, h, rows, top)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)
  txt(x + 5, y + 4, "STATIC COVERAGE", C.dim, 1)

  local mapX = x + 5
  local mapY = y + 15
  local mapW = w - 10
  local mapH = h - 20
  outline(mapX, mapY, mapW, mapH, C.border)

  local cx = mapX + math.floor(mapW / 2)
  local cy = mapY + math.floor(mapH / 2)
  line(cx, mapY + 1, cx, mapY + mapH - 2, C.grid)
  line(mapX + 1, cy, mapX + mapW - 2, cy, C.grid)

  for q = 1, 3 do
    local gx = mapX + math.floor(mapW * q / 4)
    local gy = mapY + math.floor(mapH * q / 4)
    line(gx, mapY + 1, gx, mapY + mapH - 2, C.grid)
    line(mapX + 1, gy, mapX + mapW - 2, gy, C.grid)
  end

  -- Static monitored-sector boundaries, not a rotating sweep.
  local ang = math.atan(SECTOR_Z, SECTOR_X)
  local spread = math.acos(SECTOR_COS)
  local radius = math.min(mapW, mapH) / 2 - 3
  line(cx, cy, cx + math.cos(ang - spread) * radius, cy + math.sin(ang - spread) * radius, C.yellow)
  line(cx, cy, cx + math.cos(ang + spread) * radius, cy + math.sin(ang + spread) * radius, C.yellow)

  rect(cx - 2, cy - 2, 4, 4, C.green)
  txt(cx - 11, cy + 5, "BASE", C.green, 1)

  for _, c in ipairs(rows) do
    local px = cx + clamp(c.x / RANGE, -1, 1) * (mapW / 2 - 5)
    local py = cy + clamp(c.z / RANGE, -1, 1) * (mapH / 2 - 5)
    local color = C.cyan
    if c.rank == 1 then color = C.yellow end
    if c.rank == 2 then color = C.orange end
    if c.rank == 3 then color = C.red end
    if c.rank >= 4 then color = C.magenta end
    local s = c == top and 6 or 4
    rect(px - math.floor(s / 2), py - math.floor(s / 2), s, s, color)
    if c == top then outline(px - 4, py - 4, 8, 8, color) end
  end

  txt(x + 5, y + h - 10, "RANGE +/-2048", C.dim, 1)
end

local function drawRightPanel(x, y, w, h, rows, top)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)

  txt(x + 5, y + 4, "DETECTION", C.dim, 1)
  txt(x + 5, y + 15, tostring(#rows), #rows > 0 and C.cyan or C.green, 2)
  txt(x + 5, y + 31, #rows == 1 and "CONTACT" or "CONTACTS", C.text, 1)

  line(x + 4, y + 43, x + w - 5, y + 43, C.border)
  txt(x + 5, y + 48, "PRIMARY", C.dim, 1)

  if not top then
    txt(x + 5, y + 61, "NONE", C.green, 1)
    txt(x + 5, y + 73, "NO AIRCRAFT", C.dim, 1)
  else
    local color = C.cyan
    if top.rank == 1 then color = C.yellow end
    if top.rank == 2 then color = C.orange end
    if top.rank == 3 then color = C.red end
    if top.rank >= 4 then color = C.magenta end

    txt(x + 5, y + 60, short(top.name, 9), color, 1)
    txt(x + 5, y + 72, top.state, color, 1)
    txt(x + 5, y + 84, string.format("D %.0f", top.d), C.text, 1)
    txt(x + 5, y + 96, string.format("CL %+.0f/s", top.closing), top.closing > 1 and C.orange or C.dim, 1)
    txt(x + 5, y + 108, top.eta and string.format("ETA %.0fs", top.eta) or "ETA --", C.text, 1)
  end
end

local function drawFooter(x, y, w, h, sirenOn)
  rect(x, y, w, h, C.panel2)
  outline(x, y, w, h, C.border)

  local left = sirenOn and "SIREN ACTIVE" or "SIREN STANDBY"
  txt(x + 6, y + 5, left, sirenOn and C.red or C.green, 1)
  txt(x + 6, y + 16, TEST_MODE and (QUIET_TEST and "TEST MODE / QUIET" or "TEST MODE") or "LIVE SENSOR DATA", TEST_MODE and C.yellow or C.cyan, 1)

  txt(x + w - 72, y + 5, textutils.formatTime(os.time(), true), C.text, 1)
  txt(x + w - 72, y + 16, "GPU " .. short(gpuSide, 5), C.dim, 1)
end

local function draw(rows, physicalSiren)
  local status, message, statusColor, statusBg = overall(rows)
  local top = rows[1]

  gpu.fill(C.bg)

  -- Deliberately dominant status block: one glance shows whether anything is detected.
  local margin = 4
  local headerY, headerH = 4, 34
  rect(margin, headerY, W - margin * 2, headerH, statusBg)
  outline(margin, headerY, W - margin * 2, headerH, statusColor)
  hazardRail(margin + 2, headerY + 2, W - margin * 2 - 4, 4)
  txtCentered(headerY + 8, status, statusColor, 2, margin, W - margin * 2)
  txtCentered(headerY + 25, message, C.text, 1, margin, W - margin * 2)

  local mainY = 42
  local footerH = 29
  local footerY = H - footerH - 4
  local mainH = footerY - mainY - 4
  local rightW = 64
  local gap = 4
  local leftW = W - margin * 2 - rightW - gap

  drawCoverage(margin, mainY, leftW, mainH, rows, top)
  drawRightPanel(margin + leftW + gap, mainY, rightW, mainH, rows, top)
  drawFooter(margin, footerY, W - margin * 2, footerH, physicalSiren)

  gpu.sync()
end

local lastRows = {}
local lastScan = -100

local function main()
  rs.setOutput(SIREN_SIDE, false)
  while true do
    local now = nowSeconds()
    if now - lastScan >= SCAN_INTERVAL then
      lastRows = TEST_MODE and scanTest(now) or scanReal(now)
      lastScan = now
    end

    local status, message, statusColor, statusBg, shouldAlarm = overall(lastRows)
    local physicalSiren = shouldAlarm and not (TEST_MODE and QUIET_TEST)
    rs.setOutput(SIREN_SIDE, physicalSiren)
    draw(lastRows, physicalSiren)
    sleep(0.15)
  end
end

local ok, err = pcall(main)
rs.setOutput(SIREN_SIDE, false)
if not ok then error(err, 0) end
