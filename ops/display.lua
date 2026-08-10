local args = {...}
local TEST_MODE = args[1] == "test"
local QUIET_TEST = args[2] == "quiet"

local CONTRAPTION_RANGE = 2048
local PERSON_RANGE = 1024
local SIREN_SIDE = "back"
local SCAN_INTERVAL = 0.75
local HISTORY = 8
local TRACK_TTL = 8

-- Shared origin used to convert personnel sensor world coordinates into the
-- same local coordinate system used by the sublevel sensor.
local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1903, 97, 2442

-- Monitored direction in local sensor coordinates.
local SECTOR_X, SECTOR_Z = 1881, 1632
local SECTOR_LEN = math.sqrt(SECTOR_X * SECTOR_X + SECTOR_Z * SECTOR_Z)
local SECTOR_COS = 0.70

-- Personnel warning thresholds. Normal sprinting should generally stay below
-- these values; fast vehicles / unusual movement should not.
local PERSON_MIN_DISTANCE = 64
local PERSON_MIN_SPEED = 8
local PERSON_MIN_CLOSING = 6
local PERSON_MIN_SAMPLES = 3

local function nowSeconds()
  return os.epoch("utc") / 1000
end

local gpu, gpuSide
for _, name in ipairs(peripheral.getNames()) do
  local p = peripheral.wrap(name)
  if p and type(p.refreshSize) == "function" and type(p.sync) == "function" and type(p.getSize) == "function" then
    gpu, gpuSide = p, name
    break
  end
end
assert(gpu, "display controller not found")

local W, H = 0, 0
for _ = 1, 30 do
  gpu.refreshSize()
  sleep(0.1)
  W, H = gpu.getSize()
  if type(W) == "number" and type(H) == "number" and W > 0 and H > 0 then break end
end
assert(W > 0 and H > 0, "display not detected")

gpu.setSize(64)
sleep(0.1)
W, H = gpu.getSize()

local subSensor, personSensor
for _, name in ipairs(peripheral.getNames()) do
  local p = peripheral.wrap(name)
  if p then
    if not subSensor and type(p.scanForSubLevels) == "function" then subSensor = p end
    if not personSensor and type(p.scanForPlayers) == "function" then personSensor = p end
  end
end
if not TEST_MODE then assert(subSensor, "sublevel sensor not found") end

local C = {
  bg       = 0xFF05090C,
  panel    = 0xFF0B1216,
  panel2   = 0xFF101B20,
  border   = 0xFF31515A,
  grid     = 0xFF183138,
  text     = 0xFFE7F0F2,
  dim      = 0xFF758990,
  cyan     = 0xFF49D9F2,
  blue     = 0xFF6AA6FF,
  green    = 0xFF4FE39B,
  yellow   = 0xFFF0D15A,
  orange   = 0xFFFF9D3A,
  red      = 0xFFFF5C68,
  magenta  = 0xFFFF69CF,
  clearBg  = 0xFF10291E,
  infoBg   = 0xFF10232B,
  watchBg  = 0xFF312A12,
  alertBg  = 0xFF35161A,
  critical = 0xFF48131C,
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
  txt(x + math.floor((w - textWidth(s, size)) / 2), y, s, c, size)
end

local function short(s, n)
  s = tostring(s or "UNKNOWN")
  if #s <= n then return s end
  return s:sub(1, math.max(1, n - 3)) .. "..."
end

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

local function inSector(x, z)
  local h = math.sqrt(x * x + z * z)
  if h < 1 then return false end
  return (x * SECTOR_X + z * SECTOR_Z) / (h * SECTOR_LEN) >= SECTOR_COS
end

local function localDistance(x, y, z)
  return math.sqrt(x * x + y * y + z * z)
end

local subTracks = {}
local personTracks = {}

local function ensureTrack(store, key)
  if not store[key] then store[key] = {history = {}} end
  return store[key]
end

local function addSample(track, sample, now)
  local h = track.history
  h[#h + 1] = {
    t = now,
    d = sample.d,
    x = sample.x,
    y = sample.y,
    z = sample.z,
  }
  while #h > HISTORY do table.remove(h, 1) end
  track.lastSeen = now
end

local function closingRate(h)
  if #h < 2 then return 0 end
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

local function cleanup(store, now)
  for key, tr in pairs(store) do
    if not tr.lastSeen or now - tr.lastSeen > TRACK_TTL then store[key] = nil end
  end
end

local function subKey(v)
  if type(v.id) == "table" then
    local parts = {}
    for _, k in ipairs(v.id) do parts[#parts + 1] = tostring(k) end
    if #parts > 0 then return table.concat(parts, ":") end
  elseif v.id ~= nil then
    return tostring(v.id)
  end
  return tostring(v.name or "SUB") .. "@" .. math.floor(v.x or 0) .. ":" .. math.floor(v.z or 0)
end

local function classifySub(row, samples)
  row.rank = 0
  row.state = "CONTACT"
  if row.sector then
    row.rank = 1
    row.state = "WATCH"
    local threshold = row.d > 1500 and 10 or row.d > 900 and 6 or 3
    if samples >= 4 and row.closing > threshold then
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

local function scanSubs(now)
  if not subSensor then return {} end
  local ok, raw = pcall(function() return subSensor.scanForSubLevels(CONTRAPTION_RANGE) end)
  if not ok or type(raw) ~= "table" then return {} end
  local rows = {}

  for _, v in ipairs(raw) do
    local sample = {
      x = v.x or 0,
      y = v.y or 0,
      z = v.z or 0,
      d = v.distance or localDistance(v.x or 0, v.y or 0, v.z or 0),
    }
    local key = subKey(v)
    local tr = ensureTrack(subTracks, key)
    addSample(tr, sample, now)
    local closing = closingRate(tr.history)
    local row = {
      kind = "sub",
      key = key,
      name = v.name or "CONTRAP",
      x = sample.x, y = sample.y, z = sample.z, d = sample.d,
      closing = closing,
      speed = totalSpeed(tr.history),
      eta = closing > 1 and sample.d / closing or nil,
      sector = inSector(sample.x, sample.z),
    }
    classifySub(row, #tr.history)
    rows[#rows + 1] = row
  end

  cleanup(subTracks, now)
  table.sort(rows, function(a, b)
    if a.rank ~= b.rank then return a.rank > b.rank end
    return a.d < b.d
  end)
  return rows
end

local function personKey(v, index)
  if v.username and v.username ~= "" then return tostring(v.username) end
  return "PERSON-" .. tostring(index)
end

local function scanPeople(now)
  if not personSensor then return {} end
  local ok, raw = pcall(function() return personSensor.scanForPlayers(PERSON_RANGE) end)
  if not ok or type(raw) ~= "table" then return {} end
  local rows = {}

  for i, v in ipairs(raw) do
    -- The personnel sensor returns world coordinates. Convert them so they
    -- overlay directly on the same map as the sublevel sensor.
    local lx = ORIGIN_X - (v.x or ORIGIN_X)
    local ly = ORIGIN_Y - (v.y or ORIGIN_Y)
    local lz = ORIGIN_Z - (v.z or ORIGIN_Z)
    local d = localDistance(lx, ly, lz)
    local sample = {x = lx, y = ly, z = lz, d = d}
    local key = personKey(v, i)
    local tr = ensureTrack(personTracks, key)
    addSample(tr, sample, now)
    local closing = closingRate(tr.history)
    local speed = totalSpeed(tr.history)
    local warning = #tr.history >= PERSON_MIN_SAMPLES
      and d >= PERSON_MIN_DISTANCE
      and speed >= PERSON_MIN_SPEED
      and closing >= PERSON_MIN_CLOSING

    rows[#rows + 1] = {
      kind = "person",
      key = key,
      name = v.username or "UNKNOWN",
      x = lx, y = ly, z = lz, d = d,
      closing = closing,
      speed = speed,
      eta = closing > 1 and d / closing or nil,
      warning = warning,
      state = warning and "FAST IN" or "PERSON",
      rank = warning and 1 or 0,
    }
  end

  cleanup(personTracks, now)
  table.sort(rows, function(a, b)
    if a.warning ~= b.warning then return a.warning end
    return a.d < b.d
  end)
  return rows
end

local function testData(now)
  local phase = now % 52
  local subs, people = {}, {}
  local dx = SECTOR_X / SECTOR_LEN
  local dz = SECTOR_Z / SECTOR_LEN

  if phase < 6 or phase >= 48 then
    return subs, people
  elseif phase < 12 then
    people[1] = {
      kind = "person", name = "TESTER", x = -360, y = 0, z = -260,
      d = 444, speed = 2, closing = 1, eta = nil, warning = false, rank = 0, state = "PERSON",
    }
  elseif phase < 20 then
    local d = 850 - (phase - 12) * 45
    people[1] = {
      kind = "person", name = "RUNNER", x = dx * d, y = 0, z = dz * d,
      d = d, speed = 48, closing = 45, eta = d / 45, warning = true, rank = 1, state = "FAST IN",
    }
  elseif phase < 28 then
    local d = 1700 - (phase - 20) * 70
    subs[1] = {
      kind = "sub", name = "AIRCRAFT", x = dx * d, y = 20, z = dz * d,
      d = d, speed = 73, closing = 70, eta = d / 70,
      sector = true, rank = 2, state = "INBOUND",
    }
  elseif phase < 40 then
    local d1 = 1150 - (phase - 28) * 55
    local d2 = 780 - (phase - 28) * 35
    subs[1] = {
      kind = "sub", name = "AIRCRAFT", x = dx * d1, y = 20, z = dz * d1,
      d = d1, speed = 58, closing = 55, eta = d1 / 55,
      sector = true, rank = d1 < 600 and 4 or 2, state = d1 < 600 and "CRIT" or "INBOUND",
    }
    people[1] = {
      kind = "person", name = "RIDER", x = dx * d2, y = 0, z = dz * d2,
      d = d2, speed = 38, closing = 35, eta = d2 / 35,
      warning = true, rank = 1, state = "FAST IN",
    }
  else
    local d = math.max(180, 460 - (phase - 40) * 60)
    subs[1] = {
      kind = "sub", name = "AIRCRAFT", x = dx * d, y = 20, z = dz * d,
      d = d, speed = 63, closing = 60, eta = d / 60,
      sector = true, rank = 4, state = "CRIT",
    }
  end

  return subs, people
end

local function summarize(subs, people)
  local topSub = subs[1]
  local topPerson = people[1]
  local personWarning = topPerson and topPerson.warning or false
  local subInbound = topSub and topSub.rank >= 2 or false
  local subSevere = topSub and topSub.rank >= 3 or false
  local combined = personWarning and subInbound
  local siren = combined or subSevere

  if topSub and topSub.rank >= 4 then
    return "CRITICAL", "IMMEDIATE CONTRAPTION THREAT", C.magenta, C.critical, siren, combined
  elseif combined then
    return "ALARM", "PERSON + CONTRAPTION APPROACH", C.red, C.alertBg, siren, true
  elseif topSub and topSub.rank >= 3 then
    return "HIGH", "CONTRAPTION THREAT", C.red, C.alertBg, siren, false
  elseif personWarning then
    return "WARNING", "FAST PERSON APPROACHING", C.orange, C.watchBg, false, false
  elseif subInbound then
    return "INBOUND", "CONTRAPTION APPROACHING", C.orange, C.watchBg, false, false
  elseif topSub and topSub.rank == 1 then
    return "WATCH", "CONTRAPTION IN WATCH SECTOR", C.yellow, C.watchBg, false, false
  elseif #subs > 0 or #people > 0 then
    return "CONTACT", tostring(#people) .. " PERSON / " .. tostring(#subs) .. " CONTRAP", C.cyan, C.infoBg, false, false
  else
    return "CLEAR", "NO DETECTIONS", C.green, C.clearBg, false, false
  end
end

local function drawPersonMarker(px, py, color, warning)
  line(px - 3, py, px + 3, py, color)
  line(px, py - 3, px, py + 3, color)
  rect(px - 1, py - 1, 3, 3, color)
  if warning then outline(px - 5, py - 5, 10, 10, color) end
end

local function drawCoverage(x, y, w, h, subs, people)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)
  txt(x + 5, y + 4, "COVERAGE", C.dim, 1)

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

  -- Inner box is the smaller personnel sensor range.
  local personFrac = PERSON_RANGE / CONTRAPTION_RANGE
  local pw = math.floor(mapW * personFrac)
  local ph = math.floor(mapH * personFrac)
  outline(cx - math.floor(pw / 2), cy - math.floor(ph / 2), pw, ph, C.blue)
  txt(cx - math.floor(pw / 2) + 3, cy - math.floor(ph / 2) + 3, "P1024", C.blue, 1)

  -- Static monitored-sector boundaries.
  local ang = math.atan(SECTOR_Z, SECTOR_X)
  local spread = math.acos(SECTOR_COS)
  local radius = math.min(mapW, mapH) / 2 - 3
  line(cx, cy, cx + math.cos(ang - spread) * radius, cy + math.sin(ang - spread) * radius, C.yellow)
  line(cx, cy, cx + math.cos(ang + spread) * radius, cy + math.sin(ang + spread) * radius, C.yellow)

  rect(cx - 2, cy - 2, 4, 4, C.green)

  for _, c in ipairs(subs) do
    local px = cx + clamp(c.x / CONTRAPTION_RANGE, -1, 1) * (mapW / 2 - 5)
    local py = cy + clamp(c.z / CONTRAPTION_RANGE, -1, 1) * (mapH / 2 - 5)
    local color = C.cyan
    if c.rank == 1 then color = C.yellow end
    if c.rank == 2 then color = C.orange end
    if c.rank == 3 then color = C.red end
    if c.rank >= 4 then color = C.magenta end
    rect(px - 2, py - 2, 5, 5, color)
  end

  for i, p in ipairs(people) do
    local px = cx + clamp(p.x / CONTRAPTION_RANGE, -1, 1) * (mapW / 2 - 5)
    local py = cy + clamp(p.z / CONTRAPTION_RANGE, -1, 1) * (mapH / 2 - 5)
    local color = p.warning and C.orange or C.blue
    drawPersonMarker(px, py, color, p.warning)
    if i <= 4 then txt(px + 4, py - 3, short(p.name, 5), color, 1) end
  end

  txt(x + 5, y + h - 10, "SQUARE  +/-2048", C.dim, 1)
end

local function drawSide(x, y, w, h, subs, people)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)

  txt(x + 5, y + 4, "DETECTIONS", C.dim, 1)
  txt(x + 5, y + 15, "P " .. tostring(#people), #people > 0 and C.blue or C.dim, 1)
  txt(x + 35, y + 15, "C " .. tostring(#subs), #subs > 0 and C.cyan or C.dim, 1)

  line(x + 4, y + 27, x + w - 5, y + 27, C.border)
  txt(x + 5, y + 32, "PERSON", C.dim, 1)
  local p = people[1]
  if p then
    txt(x + 5, y + 43, short(p.name, 9), p.warning and C.orange or C.blue, 1)
    txt(x + 5, y + 54, p.warning and "FAST IN" or "TRACKED", p.warning and C.orange or C.text, 1)
    txt(x + 5, y + 65, string.format("V%.0f C%+.0f", p.speed, p.closing), p.warning and C.orange or C.dim, 1)
  else
    txt(x + 5, y + 45, "NONE", C.green, 1)
  end

  line(x + 4, y + 77, x + w - 5, y + 77, C.border)
  txt(x + 5, y + 82, "CONTRAP", C.dim, 1)
  local c = subs[1]
  if c then
    local color = C.cyan
    if c.rank == 1 then color = C.yellow end
    if c.rank == 2 then color = C.orange end
    if c.rank == 3 then color = C.red end
    if c.rank >= 4 then color = C.magenta end
    txt(x + 5, y + 93, c.state, color, 1)
    txt(x + 5, y + 104, string.format("D%.0f C%+.0f", c.d, c.closing), C.text, 1)
  else
    txt(x + 5, y + 95, "NONE", C.green, 1)
  end
end

local function drawHeader(status, message, color, bg)
  local x, y, w, h = 6, 6, W - 12, 34
  rect(x, y, w, h, bg)
  outline(x, y, w, h, color)
  hazardRail(x + 4, y + 4, 24, 4)
  hazardRail(x + w - 28, y + 4, 24, 4)
  txtCentered(y + 4, status, color, 2, x, w)
  txtCentered(y + 22, message, C.text, 1, x, w)
end

local function drawFooter(sirenOn)
  local x, y, w, h = 6, H - 26, W - 12, 20
  rect(x, y, w, h, C.panel2)
  outline(x, y, w, h, C.border)
  txt(x + 5, y + 4, TEST_MODE and (QUIET_TEST and "TEST QUIET" or "TEST") or "LIVE", TEST_MODE and C.yellow or C.green, 1)
  txt(x + 55, y + 4, sirenOn and "SIREN ON" or "SIREN OFF", sirenOn and C.red or C.dim, 1)
  txt(x + 119, y + 4, (subSensor and "C+" or "C-") .. " " .. (personSensor and "P+" or "P-"), C.dim, 1)
  txt(x + 5, y + 13, "+ PERSON   # CONTRAP", C.dim, 1)
  txt(x + w - 49, y + 13, textutils.formatTime(os.time(), true), C.text, 1)
end

local function draw(subs, people, sirenOn)
  local status, message, color, bg, autoSiren, combined = summarize(subs, people)
  sirenOn = sirenOn and autoSiren

  gpu.fill(C.bg)
  drawHeader(status, message, color, bg)

  local bodyY = 46
  local footerY = H - 32
  local bodyH = footerY - bodyY
  local sideW = 66
  local gap = 6
  local mapW = W - 12 - sideW - gap
  drawCoverage(6, bodyY, mapW, bodyH, subs, people)
  drawSide(6 + mapW + gap, bodyY, sideW, bodyH, subs, people)
  drawFooter(sirenOn)
  gpu.sync()
end

local function main()
  rs.setOutput(SIREN_SIDE, false)
  while true do
    local now = nowSeconds()
    local subs, people
    if TEST_MODE then
      subs, people = testData(now)
    else
      subs = scanSubs(now)
      people = scanPeople(now)
    end

    local _, _, _, _, shouldSiren = summarize(subs, people)
    local physicalSiren = shouldSiren and not (TEST_MODE and QUIET_TEST)
    rs.setOutput(SIREN_SIDE, physicalSiren)
    draw(subs, people, physicalSiren)
    sleep(SCAN_INTERVAL)
  end
end

local ok, err = pcall(main)
rs.setOutput(SIREN_SIDE, false)
if not ok then error(err, 0) end
