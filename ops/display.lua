local args = {...}
local TEST_MODE = args[1] == "test"
local QUIET_TEST = args[2] == "quiet"

local CONTRAPTION_RANGE = 2048
local PERSON_RANGE = 1024
local SIREN_SIDE = "back"
local SCAN_INTERVAL = 0.75
local HISTORY = 8
local TRACK_TTL = 8

-- Personnel sensor world origin, aligned with the sublevel sensor local frame.
local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1903, 97, 2442

-- Monitored direction in local sensor coordinates.
local SECTOR_X, SECTOR_Z = 1881, 1632
local SECTOR_LEN = math.sqrt(SECTOR_X * SECTOR_X + SECTOR_Z * SECTOR_Z)
local SECTOR_COS = 0.70

-- Fast-person warning thresholds.
local PERSON_MIN_DISTANCE = 64
local PERSON_MIN_SPEED = 8
local PERSON_MIN_CLOSING = 6
local PERSON_MIN_SAMPLES = 3

-- 3x3 wall list behavior. Every contact is always plotted on the map; the
-- detail lists page automatically when there are too many rows to fit.
local LIST_ROWS = 3
local LIST_PAGE_SECONDS = 3

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
  h[#h + 1] = {t = now, d = sample.d, x = sample.x, y = sample.y, z = sample.z}
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
  local phase = now % 56
  local subs, people = {}, {}
  local dx = SECTOR_X / SECTOR_LEN
  local dz = SECTOR_Z / SECTOR_LEN

  if phase < 6 or phase >= 52 then
    return subs, people
  end

  people[1] = {kind="person", name="ALPHA", x=-420, y=0, z=-260, d=494, speed=2, closing=1, warning=false, rank=0, state="PERSON"}
  people[2] = {kind="person", name="BRAVO", x=260, y=0, z=-580, d=636, speed=3, closing=-1, warning=false, rank=0, state="PERSON"}
  people[3] = {kind="person", name="CHARLIE", x=-180, y=0, z=340, d=385, speed=1, closing=0, warning=false, rank=0, state="PERSON"}
  people[4] = {kind="person", name="DELTA", x=520, y=0, z=220, d=565, speed=2, closing=0, warning=false, rank=0, state="PERSON"}
  people[5] = {kind="person", name="ECHO", x=-610, y=0, z=120, d=622, speed=2, closing=0, warning=false, rank=0, state="PERSON"}

  subs[1] = {kind="sub", name="CARGO", x=-1100, y=10, z=-720, d=1315, speed=0, closing=0, eta=nil, sector=false, rank=0, state="CONTACT"}
  subs[2] = {kind="sub", name="SCOUT", x=720, y=15, z=-980, d=1217, speed=0, closing=0, eta=nil, sector=false, rank=0, state="CONTACT"}
  subs[3] = {kind="sub", name="HAULER", x=-820, y=5, z=1030, d=1317, speed=0, closing=0, eta=nil, sector=false, rank=0, state="CONTACT"}
  subs[4] = {kind="sub", name="DRONE", x=930, y=25, z=930, d=1316, speed=0, closing=0, eta=nil, sector=true, rank=1, state="WATCH"}
  subs[5] = {kind="sub", name="AIR-2", x=1120, y=25, z=970, d=1482, speed=0, closing=0, eta=nil, sector=true, rank=1, state="WATCH"}

  if phase >= 14 then
    local d = 850 - math.min(10, phase - 14) * 45
    people[1].x, people[1].z = dx * d, dz * d
    people[1].d = d
    people[1].speed, people[1].closing = 48, 45
    people[1].eta = d / 45
    people[1].warning, people[1].rank, people[1].state = true, 1, "FAST IN"
  end

  if phase >= 24 then
    local d = 1700 - math.min(12, phase - 24) * 70
    subs[4].x, subs[4].z = dx * d, dz * d
    subs[4].d = d
    subs[4].speed, subs[4].closing, subs[4].eta = 73, 70, d / 70
    subs[4].rank, subs[4].state = d < 600 and 4 or (d < 1200 and 3 or 2), d < 600 and "CRIT" or (d < 1200 and "HIGH" or "INBOUND")
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
    return "CRITICAL", "IMMEDIATE CONTRAPTION THREAT", C.magenta, C.critical, siren
  elseif combined then
    return "ALARM", "PERSON + CONTRAPTION APPROACH", C.red, C.alertBg, siren
  elseif topSub and topSub.rank >= 3 then
    return "HIGH", "CONTRAPTION THREAT", C.red, C.alertBg, siren
  elseif personWarning then
    return "WARNING", "FAST PERSON APPROACHING", C.orange, C.watchBg, false
  elseif subInbound then
    return "INBOUND", "CONTRAPTION APPROACHING", C.orange, C.watchBg, false
  elseif topSub and topSub.rank == 1 then
    return "WATCH", "CONTRAPTION IN WATCH SECTOR", C.yellow, C.watchBg, false
  elseif #subs > 0 or #people > 0 then
    return "CONTACT", tostring(#people) .. " PERSON / " .. tostring(#subs) .. " CONTRAP", C.cyan, C.infoBg, false
  else
    return "CLEAR", "NO DETECTIONS", C.green, C.clearBg, false
  end
end

local function subColor(c)
  if c.rank >= 4 then return C.magenta end
  if c.rank == 3 then return C.red end
  if c.rank == 2 then return C.orange end
  if c.rank == 1 then return C.yellow end
  return C.cyan
end

local function drawPersonMarker(px, py, color, warning)
  line(px - 3, py, px + 3, py, color)
  line(px, py - 3, px, py + 3, color)
  rect(px - 1, py - 1, 3, 3, color)
  if warning then outline(px - 5, py - 5, 10, 10, color) end
end

local COLLISION_OFFSETS = {
  {0,0}, {5,0}, {-5,0}, {0,5}, {0,-5},
  {5,5}, {-5,5}, {5,-5}, {-5,-5}, {8,0}, {-8,0},
}

local function reserveMarker(px, py, occupied, minX, minY, maxX, maxY)
  local bx, by = math.floor(px + 0.5), math.floor(py + 0.5)
  for _, off in ipairs(COLLISION_OFFSETS) do
    local x = clamp(bx + off[1], minX, maxX)
    local y = clamp(by + off[2], minY, maxY)
    local key = tostring(math.floor(x / 4)) .. ":" .. tostring(math.floor(y / 4))
    if not occupied[key] then
      occupied[key] = true
      return x, y
    end
  end
  return bx, by
end

local function drawCoverage(x, y, w, h, subs, people)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)
  txt(x + 5, y + 4, "ALL TRACKS", C.dim, 1)

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

  local personFrac = PERSON_RANGE / CONTRAPTION_RANGE
  local pw = math.floor(mapW * personFrac)
  local ph = math.floor(mapH * personFrac)
  outline(cx - math.floor(pw / 2), cy - math.floor(ph / 2), pw, ph, C.blue)
  txt(cx - math.floor(pw / 2) + 3, cy - math.floor(ph / 2) + 3, "P1024", C.blue, 1)

  local ang = math.atan(SECTOR_Z, SECTOR_X)
  local spread = math.acos(SECTOR_COS)
  local radius = math.min(mapW, mapH) / 2 - 3
  line(cx, cy, cx + math.cos(ang - spread) * radius, cy + math.sin(ang - spread) * radius, C.yellow)
  line(cx, cy, cx + math.cos(ang + spread) * radius, cy + math.sin(ang + spread) * radius, C.yellow)

  rect(cx - 2, cy - 2, 4, 4, C.green)

  local occupied = {}
  local minPX, minPY = mapX + 4, mapY + 4
  local maxPX, maxPY = mapX + mapW - 5, mapY + mapH - 5

  for i, c in ipairs(subs) do
    local rawX = cx + clamp(c.x / CONTRAPTION_RANGE, -1, 1) * (mapW / 2 - 5)
    local rawY = cy + clamp(c.z / CONTRAPTION_RANGE, -1, 1) * (mapH / 2 - 5)
    local px, py = reserveMarker(rawX, rawY, occupied, minPX, minPY, maxPX, maxPY)
    local color = subColor(c)
    rect(px - 2, py - 2, 5, 5, color)
    if i <= 9 then txt(px + 4, py - 3, "C" .. tostring(i), color, 1) end
  end

  for i, p in ipairs(people) do
    local rawX = cx + clamp(p.x / CONTRAPTION_RANGE, -1, 1) * (mapW / 2 - 5)
    local rawY = cy + clamp(p.z / CONTRAPTION_RANGE, -1, 1) * (mapH / 2 - 5)
    local px, py = reserveMarker(rawX, rawY, occupied, minPX, minPY, maxPX, maxPY)
    local color = p.warning and C.orange or C.blue
    drawPersonMarker(px, py, color, p.warning)
    if i <= 9 then txt(px + 4, py - 3, "P" .. tostring(i), color, 1) end
  end

  txt(x + 5, y + h - 10, "P+  C#   +/-2048", C.dim, 1)
end

local function pageInfo(count, now)
  if count <= LIST_ROWS then return 1, 1, 1, count end
  local pages = math.ceil(count / LIST_ROWS)
  local page = (math.floor(now / LIST_PAGE_SECONDS) % pages) + 1
  local first = (page - 1) * LIST_ROWS + 1
  local last = math.min(count, first + LIST_ROWS - 1)
  return page, pages, first, last
end

local function drawTrackList(x, y, w, title, prefix, rows, now, isPerson)
  local page, pages, first, last = pageInfo(#rows, now)
  txt(x + 4, y, title .. " " .. tostring(#rows), C.dim, 1)
  if pages > 1 then txt(x + w - 25, y, tostring(page) .. "/" .. tostring(pages), C.dim, 1) end

  if #rows == 0 then
    txt(x + 4, y + 12, "NONE", C.green, 1)
    return
  end

  local rowY = y + 11
  for i = first, last do
    local r = rows[i]
    local localIndex = i - first
    local yy = rowY + localIndex * 9
    local color
    if isPerson then
      color = r.warning and C.orange or C.blue
    else
      color = subColor(r)
    end

    txt(x + 4, yy, prefix .. tostring(i), color, 1)
    if isPerson then
      txt(x + 19, yy, short(r.name, 7), color, 1)
      txt(x + w - 25, yy, tostring(math.floor(r.d + 0.5)), C.dim, 1)
    else
      txt(x + 19, yy, short(r.state, 6), color, 1)
      txt(x + w - 25, yy, tostring(math.floor(r.d + 0.5)), C.dim, 1)
    end
  end
end

local function drawSide(x, y, w, h, subs, people, now)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)

  txt(x + 5, y + 4, "DETECTIONS", C.text, 1)
  txt(x + 5, y + 15, "P " .. tostring(#people), #people > 0 and C.blue or C.dim, 1)
  txt(x + 38, y + 15, "C " .. tostring(#subs), #subs > 0 and C.cyan or C.dim, 1)
  line(x + 4, y + 27, x + w - 5, y + 27, C.border)

  local sectionY = y + 32
  drawTrackList(x, sectionY, w, "PLAYERS", "P", people, now, true)

  local dividerY = sectionY + 40
  line(x + 4, dividerY, x + w - 5, dividerY, C.border)
  drawTrackList(x, dividerY + 5, w, "CONTRAPS", "C", subs, now, false)
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
  txt(x + 5, y + 13, "ALL MAP / LISTS PAGE", C.dim, 1)
  txt(x + w - 49, y + 13, textutils.formatTime(os.time(), true), C.text, 1)
end

local function draw(subs, people, sirenOn, now)
  local status, message, color, bg, autoSiren = summarize(subs, people)
  sirenOn = sirenOn and autoSiren

  gpu.fill(C.bg)
  drawHeader(status, message, color, bg)

  local bodyY = 46
  local footerY = H - 32
  local bodyH = footerY - bodyY
  local sideW = 78
  local gap = 6
  local mapW = W - 12 - sideW - gap
  drawCoverage(6, bodyY, mapW, bodyH, subs, people)
  drawSide(6 + mapW + gap, bodyY, sideW, bodyH, subs, people, now)
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
      table.sort(subs, function(a, b)
        if a.rank ~= b.rank then return a.rank > b.rank end
        return a.d < b.d
      end)
      table.sort(people, function(a, b)
        if a.warning ~= b.warning then return a.warning end
        return a.d < b.d
      end)
    else
      subs = scanSubs(now)
      people = scanPeople(now)
    end

    local _, _, _, _, shouldSiren = summarize(subs, people)
    local physicalSiren = shouldSiren and not (TEST_MODE and QUIET_TEST)
    rs.setOutput(SIREN_SIDE, physicalSiren)
    draw(subs, people, physicalSiren, now)
    sleep(SCAN_INTERVAL)
  end
end

local ok, err = pcall(main)
rs.setOutput(SIREN_SIDE, false)
if not ok then error(err, 0) end