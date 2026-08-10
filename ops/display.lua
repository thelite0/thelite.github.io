local args = {...}
local START_TEST = args[1] == "test"
local QUIET_TEST = START_TEST and args[2] == "quiet"

local CONTRAPTION_RANGE = 2048
local PERSON_RANGE = 1024
local SIREN_SIDE = "back"
local SCAN_INTERVAL = 0.75
local FRAME_INTERVAL = 0.25
local HISTORY = 10
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

-- Inbound contraption thresholds. Inbound is intentionally NOT sector-limited:
-- anything closing on the base fast enough is an alarm condition.
local SUB_MIN_SAMPLES = 4

-- Operator controls.
local MUTE_SECONDS = 45
local LIST_ROWS = 7

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
if not START_TEST then assert(subSensor, "sublevel sensor not found") end

local C = {
  bg       = 0xFF05090C,
  panel    = 0xFF0B1216,
  panel2   = 0xFF101B20,
  panel3   = 0xFF0A1013,
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
  button   = 0xFF16242A,
  buttonOn = 0xFF234552,
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

local function rect(x, y, w, h, color)
  x, y, w, h = math.floor(x), math.floor(y), math.floor(w), math.floor(h)
  if w < 1 or h < 1 then return end
  x = clamp(x, MIN_X, MAX_X)
  y = clamp(y, MIN_Y, MAX_Y)
  w = math.min(w, MAX_X - x + 1)
  h = math.min(h, MAX_Y - y + 1)
  if w >= 1 and h >= 1 then gpu.filledRectangle(x, y, w, h, color) end
end

local function outline(x, y, w, h, color)
  x, y, w, h = math.floor(x), math.floor(y), math.floor(w), math.floor(h)
  x = clamp(x, MIN_X, MAX_X)
  y = clamp(y, MIN_Y, MAX_Y)
  w = math.min(w, MAX_X - x + 1)
  h = math.min(h, MAX_Y - y + 1)
  if w >= 2 and h >= 2 then gpu.rectangle(x, y, w, h, color) end
end

local function line(x1, y1, x2, y2, color)
  x1 = clamp(math.floor(x1), MIN_X, MAX_X)
  y1 = clamp(math.floor(y1), MIN_Y, MAX_Y)
  x2 = clamp(math.floor(x2), MIN_X, MAX_X)
  y2 = clamp(math.floor(y2), MIN_Y, MAX_Y)
  gpu.lineS(x1, y1, x2, y2, color)
end

local function txt(x, y, s, color, size)
  x = clamp(math.floor(x), MIN_X, MAX_X - 2)
  y = clamp(math.floor(y), MIN_Y, MAX_Y - 8)
  pcall(gpu.drawText, x, y, tostring(s), color or C.text, 0x00000000, size or 1)
end

-- Lua's # operator counts UTF-8 bytes, not visible characters. The UI is
-- Russian, so centering and truncation need simple UTF-8-aware helpers.
local function utf8Len(s)
  s = tostring(s or "")
  local n = 0
  for i = 1, #s do
    local b = s:byte(i)
    if b < 0x80 or b >= 0xC0 then n = n + 1 end
  end
  return n
end

local function utf8Sub(s, first, last)
  s = tostring(s or "")
  local starts = {}
  for i = 1, #s do
    local b = s:byte(i)
    if b < 0x80 or b >= 0xC0 then starts[#starts + 1] = i end
  end
  if #starts == 0 then return "" end
  first = math.max(1, first or 1)
  last = math.min(#starts, last or #starts)
  if first > last then return "" end
  local a = starts[first]
  local b = last < #starts and (starts[last + 1] - 1) or #s
  return s:sub(a, b)
end

local function textWidth(s, size)
  return utf8Len(s) * 6 * (size or 1)
end

local function txtCentered(y, s, color, size, x, w)
  x = x or 0
  w = w or W
  txt(x + math.floor((w - textWidth(s, size)) / 2), y, s, color, size)
end

local function short(s, n)
  s = tostring(s or "НЕИЗВ.")
  if utf8Len(s) <= n then return s end
  return utf8Sub(s, 1, math.max(1, n - 3)) .. "..."
end

local function stateText(s)
  local m = {
    CONTACT = "КОНТАКТ",
    WATCH = "НАБЛЮД.",
    INBOUND = "СБЛИЖ.",
    HIGH = "УГРОЗА",
    CRIT = "КРИТ.",
    PERSON = "ИГРОК",
    ["FAST IN"] = "БЫСТР. ВХОД",
    TRACK = "ЦЕЛЬ",
  }
  return m[s] or tostring(s or "ЦЕЛЬ")
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

local function subColor(c)
  if c.rank >= 4 then return C.magenta end
  if c.rank == 3 then return C.red end
  if c.rank == 2 then return C.orange end
  if c.rank == 1 then return C.yellow end
  return C.cyan
end

local subTracks = {}
local personTracks = {}
local liveSubs, livePeople = {}, {}
local lastSubScan, lastPersonScan = nil, nil
local lastSubError, lastPersonError = nil, nil

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
  row.rank = row.sector and 1 or 0
  row.state = row.sector and "WATCH" or "CONTACT"

  local threshold = row.d > 1500 and 10 or row.d > 900 and 6 or 3
  if samples >= SUB_MIN_SAMPLES and row.closing > threshold then
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

local function scanSubs(now)
  if not subSensor then
    lastSubError = "НЕТ ДАТЧИКА"
    return {}
  end

  local ok, raw = pcall(function() return subSensor.scanForSubLevels(CONTRAPTION_RANGE) end)
  if not ok or type(raw) ~= "table" then
    lastSubError = ok and "ОШИБКА ОТВЕТА" or tostring(raw)
    return {}
  end

  lastSubScan = now
  lastSubError = nil
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
      name = v.name or "АППАРАТ",
      x = sample.x, y = sample.y, z = sample.z, d = sample.d,
      closing = closing,
      speed = totalSpeed(tr.history),
      eta = closing > 1 and sample.d / closing or nil,
      sector = inSector(sample.x, sample.z),
      history = tr.history,
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
  if not personSensor then
    lastPersonError = "НЕТ ДАТЧИКА"
    return {}
  end

  local ok, raw = pcall(function() return personSensor.scanForPlayers(PERSON_RANGE) end)
  if not ok or type(raw) ~= "table" then
    lastPersonError = ok and "ОШИБКА ОТВЕТА" or tostring(raw)
    return {}
  end

  lastPersonScan = now
  lastPersonError = nil
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
      name = v.username or "НЕИЗВ.",
      x = lx, y = ly, z = lz, d = d,
      worldX = v.x, worldY = v.y, worldZ = v.z,
      health = v.health, maxHealth = v.maxHealth,
      closing = closing,
      speed = speed,
      eta = closing > 1 and d / closing or nil,
      warning = warning,
      state = warning and "FAST IN" or "PERSON",
      rank = warning and 1 or 0,
      history = tr.history,
    }
  end

  cleanup(personTracks, now)
  table.sort(rows, function(a, b)
    if a.warning ~= b.warning then return a.warning end
    return a.d < b.d
  end)
  return rows
end

-- Manual simulation scenarios used by the interactive TEST page.
local simScenario = START_TEST and "NONE" or nil
local simStart = nowSeconds()

local function simData(now)
  if not simScenario or simScenario == "NONE" then return {}, {} end

  local t = now - simStart
  local dx = SECTOR_X / SECTOR_LEN
  local dz = SECTOR_Z / SECTOR_LEN
  local subs, people = {}, {}

  if simScenario == "PLAYER" then
    people = {
      {kind="person", key="SIM-P1", name="ALPHA", x=-420, y=0, z=-260, d=494, speed=2, closing=1, warning=false, rank=0, state="PERSON"},
      {kind="person", key="SIM-P2", name="BRAVO", x=260, y=0, z=-580, d=636, speed=3, closing=-1, warning=false, rank=0, state="PERSON"},
      {kind="person", key="SIM-P3", name="CHARLIE", x=-180, y=0, z=340, d=385, speed=1, closing=0, warning=false, rank=0, state="PERSON"},
    }
  elseif simScenario == "FAST" then
    local d = math.max(120, 900 - (t % 14) * 55)
    people = {
      {kind="person", key="SIM-P1", name="RUNNER", x=dx*d, y=0, z=dz*d, d=d, speed=58, closing=55, eta=d/55, warning=true, rank=1, state="FAST IN"},
      {kind="person", key="SIM-P2", name="STATIC", x=-300, y=0, z=440, d=533, speed=0, closing=0, warning=false, rank=0, state="PERSON"},
    }
  elseif simScenario == "CONTRAP" then
    subs = {
      {kind="sub", key="SIM-C1", name="CARGO", x=-1100, y=10, z=-720, d=1315, speed=0, closing=0, eta=nil, sector=false, rank=0, state="CONTACT"},
      {kind="sub", key="SIM-C2", name="WATCHER", x=980, y=20, z=850, d=1297, speed=0, closing=0, eta=nil, sector=true, rank=1, state="WATCH"},
    }
  elseif simScenario == "INBOUND" then
    local d = math.max(260, 1900 - (t % 22) * 72)
    subs = {
      {kind="sub", key="SIM-C1", name="AIRCRAFT", x=dx*d, y=20, z=dz*d, d=d, speed=75, closing=72, eta=d/72, sector=true, rank=d<550 and 4 or (d<1200 and 3 or 2), state=d<550 and "CRIT" or (d<1200 and "HIGH" or "INBOUND")},
    }
  elseif simScenario == "COMBINED" then
    local dc = math.max(320, 1700 - (t % 18) * 70)
    local dp = math.max(120, 820 - (t % 14) * 45)
    subs = {
      {kind="sub", key="SIM-C1", name="AIRCRAFT", x=dx*dc, y=25, z=dz*dc, d=dc, speed=73, closing=70, eta=dc/70, sector=true, rank=dc<550 and 4 or (dc<1200 and 3 or 2), state=dc<550 and "CRIT" or (dc<1200 and "HIGH" or "INBOUND")},
    }
    people = {
      {kind="person", key="SIM-P1", name="RIDER", x=dx*dp, y=0, z=dz*dp, d=dp, speed=48, closing=45, eta=dp/45, warning=true, rank=1, state="FAST IN"},
    }
  elseif simScenario == "CRIT" then
    subs = {
      {kind="sub", key="SIM-C1", name="THREAT", x=260, y=18, z=220, d=341, speed=66, closing=62, eta=5.5, sector=true, rank=4, state="CRIT"},
    }
  end

  table.sort(subs, function(a, b)
    if a.rank ~= b.rank then return a.rank > b.rank end
    return a.d < b.d
  end)
  table.sort(people, function(a, b)
    if a.warning ~= b.warning then return a.warning end
    return a.d < b.d
  end)
  return subs, people
end

local function findThreats(subs, people)
  local topSub = subs[1]
  local fastPerson = nil
  local inboundSub = nil
  local severeSub = nil
  local critSub = nil

  for _, p in ipairs(people) do
    if p.warning then fastPerson = p break end
  end
  for _, c in ipairs(subs) do
    if c.rank >= 4 and not critSub then critSub = c end
    if c.rank >= 3 and not severeSub then severeSub = c end
    if c.rank >= 2 and not inboundSub then inboundSub = c end
  end

  return topSub, fastPerson, inboundSub, severeSub, critSub
end

local function summarize(subs, people)
  local topSub, fastPerson, inboundSub, severeSub, critSub = findThreats(subs, people)
  local combined = fastPerson and inboundSub

  if critSub then
    return "КРИТИЧНО", "КРИТИЧЕСКАЯ УГРОЗА", C.magenta, C.critical, true, 5, critSub
  elseif combined then
    return "ТРЕВОГА", "ИГРОК + АППАРАТ СБЛИЖАЮТСЯ", C.red, C.alertBg, true, 4, inboundSub
  elseif severeSub then
    return "УГРОЗА", "ОПАСНЫЙ АППАРАТ", C.red, C.alertBg, true, 4, severeSub
  elseif inboundSub then
    return "СБЛИЖЕНИЕ", "АППАРАТ ПРИБЛИЖАЕТСЯ", C.orange, C.alertBg, true, 3, inboundSub
  elseif fastPerson then
    return "ВНИМАНИЕ", "БЫСТРЫЙ ИГРОК ПРИБЛИЖАЕТСЯ", C.orange, C.watchBg, false, 2, fastPerson
  elseif topSub and topSub.rank == 1 then
    return "НАБЛЮД.", "АППАРАТ В СЕКТОРЕ НАБЛЮДЕНИЯ", C.yellow, C.watchBg, false, 1, topSub
  elseif #subs > 0 or #people > 0 then
    return "КОНТАКТ", "ИГРОКИ " .. tostring(#people) .. " / АППАРАТЫ " .. tostring(#subs), C.cyan, C.infoBg, false, 0, topSub or people[1]
  else
    return "ЧИСТО", "КОНТАКТОВ НЕТ", C.green, C.clearBg, false, 0, nil
  end
end

-- UI state.
local page = START_TEST and "test" or "main" -- main, sys, test
local filter = "all" -- all, people, subs
local mapRange = CONTRAPTION_RANGE
local selectedKind, selectedKey = nil, nil
local listOffset = 1
local ackStatus = nil
local muteUntil = 0
local muteSeverity = 0
local notice, noticeUntil = nil, 0
local hitTargets = {}

local function setNotice(s, seconds)
  notice = s
  noticeUntil = nowSeconds() + (seconds or 2.5)
end

local function addHit(id, x, y, w, h, data)
  hitTargets[#hitTargets + 1] = {id=id, x=x, y=y, w=w, h=h, data=data}
end

local function drawButton(id, x, y, w, h, label, active, color)
  rect(x, y, w, h, active and C.buttonOn or C.button)
  outline(x, y, w, h, active and (color or C.cyan) or C.border)
  txtCentered(y + 3, label, active and (color or C.cyan) or C.text, 1, x, w)
  addHit(id, x, y, w, h)
end

local function selectedRow(subs, people)
  if not selectedKey or not selectedKind then return nil end
  local rows = selectedKind == "sub" and subs or people
  for _, r in ipairs(rows) do
    if tostring(r.key) == tostring(selectedKey) then return r end
  end
  return nil
end

local function visibleRows(subs, people)
  local rows = {}
  if filter == "all" or filter == "people" then
    for _, p in ipairs(people) do rows[#rows + 1] = p end
  end
  if filter == "all" or filter == "subs" then
    for _, c in ipairs(subs) do rows[#rows + 1] = c end
  end
  table.sort(rows, function(a, b)
    local ar = a.kind == "sub" and (a.rank + 2) or (a.warning and 3 or 0)
    local br = b.kind == "sub" and (b.rank + 2) or (b.warning and 3 or 0)
    if ar ~= br then return ar > br end
    return a.d < b.d
  end)
  return rows
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

local function drawPersonMarker(px, py, color, warning, selected)
  line(px - 3, py, px + 3, py, color)
  line(px, py - 3, px, py + 3, color)
  rect(px - 1, py - 1, 3, 3, color)
  if warning then outline(px - 5, py - 5, 10, 10, color) end
  if selected then outline(px - 7, py - 7, 14, 14, C.text) end
end

local function drawTrail(row, cx, cy, mapW, mapH, range, mapX, mapY)
  local h = row and row.history
  if type(h) ~= "table" or #h < 2 then return end
  local lastX, lastY = nil, nil
  for _, p in ipairs(h) do
    local px = cx + clamp(p.x / range, -1, 1) * (mapW / 2 - 5)
    local py = cy + clamp(p.z / range, -1, 1) * (mapH / 2 - 5)
    px = clamp(px, mapX + 3, mapX + mapW - 4)
    py = clamp(py, mapY + 3, mapY + mapH - 4)
    if lastX then line(lastX, lastY, px, py, C.dim) end
    lastX, lastY = px, py
  end
end

local function drawCoverage(x, y, w, h, subs, people)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)
  txt(x + 5, y + 4, "ТАКТИЧЕСКАЯ КАРТА", C.dim, 1)
  txt(x + w - 48, y + 4, "+/-" .. tostring(mapRange), C.dim, 1)

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

  if mapRange >= PERSON_RANGE then
    local frac = PERSON_RANGE / mapRange
    local pw = math.floor(mapW * frac)
    local ph = math.floor(mapH * frac)
    outline(cx - math.floor(pw / 2), cy - math.floor(ph / 2), pw, ph, C.blue)
  end

  local ang = math.atan(SECTOR_Z, SECTOR_X)
  local spread = math.acos(SECTOR_COS)
  local radius = math.min(mapW, mapH) / 2 - 3
  line(cx, cy, cx + math.cos(ang - spread) * radius, cy + math.sin(ang - spread) * radius, C.yellow)
  line(cx, cy, cx + math.cos(ang + spread) * radius, cy + math.sin(ang + spread) * radius, C.yellow)

  rect(cx - 2, cy - 2, 4, 4, C.green)

  local selected = selectedRow(subs, people)
  if selected then drawTrail(selected, cx, cy, mapW, mapH, mapRange, mapX, mapY) end

  local rows = visibleRows(subs, people)
  local occupied = {}
  local minPX, minPY = mapX + 4, mapY + 4
  local maxPX, maxPY = mapX + mapW - 5, mapY + mapH - 5

  for i, r in ipairs(rows) do
    if math.abs(r.x) <= mapRange and math.abs(r.z) <= mapRange then
      local rawX = cx + clamp(r.x / mapRange, -1, 1) * (mapW / 2 - 5)
      local rawY = cy + clamp(r.z / mapRange, -1, 1) * (mapH / 2 - 5)
      local px, py = reserveMarker(rawX, rawY, occupied, minPX, minPY, maxPX, maxPY)
      local isSelected = selectedKind == r.kind and tostring(selectedKey) == tostring(r.key)
      local color = r.kind == "person" and (r.warning and C.orange or C.blue) or subColor(r)

      if r.kind == "person" then
        drawPersonMarker(px, py, color, r.warning, isSelected)
      else
        rect(px - 2, py - 2, 5, 5, color)
        if isSelected then outline(px - 6, py - 6, 12, 12, C.text) end
      end

      if isSelected then
        local lx = px + 6
        if lx + 48 > mapX + mapW then lx = px - 48 end
        txt(lx, py - 3, short(r.name, 8), color, 1)
        line(cx, cy, px, py, C.dim)
      elseif (r.displayIndex or i) <= 9 then
        local label = (r.kind == "person" and "И" or "А") .. tostring(r.displayIndex or i)
        local lx = px + 4
        if lx + textWidth(label, 1) > mapX + mapW then lx = px - textWidth(label, 1) - 4 end
        txt(lx, py - 3, label, color, 1)
      end

      addHit("track", px - 6, py - 6, 12, 12, {kind=r.kind, key=r.key})
    end
  end
end

local function drawListPanel(x, y, w, h, subs, people)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)

  local selected = selectedRow(subs, people)
  if selected then
    txt(x + 5, y + 4, selected.kind == "person" and "ИГРОК" or "АППАРАТ", C.text, 1)
    drawButton("back", x + w - 34, y + 3, 29, 12, "НАЗАД", false, C.cyan)

    local color = selected.kind == "person" and (selected.warning and C.orange or C.blue) or subColor(selected)
    txt(x + 5, y + 20, short(selected.name, 10), color, 1)
    txt(x + 5, y + 32, stateText(selected.state), color, 1)
    txt(x + 5, y + 44, string.format("Д %.0f", selected.d or 0), C.text, 1)
    txt(x + 5, y + 55, string.format("СК %.1f", selected.speed or 0), C.text, 1)
    txt(x + 5, y + 66, string.format("СБ %+.1f", selected.closing or 0), (selected.closing or 0) > 1 and C.orange or C.dim, 1)
    txt(x + 5, y + 77, selected.eta and string.format("ETA %.0fs", selected.eta) or "ETA --", C.text, 1)
    txt(x + 5, y + 88, string.format("Л%.0f/%.0f", selected.x or 0, selected.z or 0), C.dim, 1)

    if selected.kind == "person" then
      if selected.worldX then txt(x + 5, y + 99, string.format("М%.0f/%.0f", selected.worldX, selected.worldZ), C.dim, 1) end
    else
      txt(x + 5, y + 99, selected.sector and "СЕКТОР ДА" or "СЕКТОР НЕТ", selected.sector and C.yellow or C.dim, 1)
    end
    return
  end

  local rows = visibleRows(subs, people)
  local maxOffset = math.max(1, #rows - LIST_ROWS + 1)
  listOffset = clamp(listOffset, 1, maxOffset)

  txt(x + 5, y + 4, "ЦЕЛИ " .. tostring(#rows), C.text, 1)
  drawButton("list_up", x + w - 45, y + 3, 18, 12, "ВВ", false, C.cyan)
  drawButton("list_down", x + w - 24, y + 3, 19, 12, "ВН", false, C.cyan)

  local yy = y + 20
  for i = listOffset, math.min(#rows, listOffset + LIST_ROWS - 1) do
    local r = rows[i]
    local color = r.kind == "person" and (r.warning and C.orange or C.blue) or subColor(r)
    local prefix = r.kind == "person" and "И" or "А"
    txt(x + 4, yy, prefix .. tostring(r.displayIndex or i), color, 1)
    txt(x + 19, yy, short(r.name, 4), color, 1)
    txt(x + w - 22, yy, tostring(math.floor(r.d + 0.5)), C.dim, 1)
    addHit("track", x + 2, yy - 2, w - 4, 10, {kind=r.kind, key=r.key})
    yy = yy + 12
  end

  if #rows == 0 then
    txt(x + 5, y + 30, "ЦЕЛЕЙ НЕТ", C.green, 1)
  end
end

local function drawHeader(status, message, color, bg, shouldSiren, physicalSiren, severity, focus)
  local x, y, w, h = 6, 6, W - 12, 34
  rect(x, y, w, h, bg)
  outline(x, y, w, h, color)
  hazardRail(x + 4, y + 4, 24, 4)
  hazardRail(x + w - 28, y + 4, 24, 4)
  txtCentered(y + 4, status, color, 2, x, w)
  txtCentered(y + 22, message, C.text, 1, x, w)

  if shouldSiren then
    txt(x + 5, y + 22, physicalSiren and "СИРЕНА" or "ТИХО", physicalSiren and C.red or C.yellow, 1)
  elseif ackStatus == status then
    txt(x + 5, y + 22, "ПРИН.", C.green, 1)
  end

  if simScenario then txt(x + w - 30, y + 22, "СИМ", C.yellow, 1) end
  addHit("header", x, y, w, h, focus and {kind=focus.kind, key=focus.key} or nil)
end

local function drawMain(subs, people, status, message, color, bg, shouldSiren, physicalSiren, severity, focus, now)
  local bodyY = 46
  local bodyH = H - 82
  local sideW = 66
  local gap = 6
  local mapW = W - 12 - sideW - gap

  drawHeader(status, message, color, bg, shouldSiren, physicalSiren, severity, focus)
  drawCoverage(6, bodyY, mapW, bodyH, subs, people)
  drawListPanel(6 + mapW + gap, bodyY, sideW, bodyH, subs, people)
end

local function drawSystem(subs, people, status, message, color, bg, shouldSiren, physicalSiren, severity, focus, now)
  drawHeader(status, message, color, bg, shouldSiren, physicalSiren, severity, focus)
  local x, y, w, h = 6, 46, W - 12, H - 82
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)
  txt(x + 6, y + 5, "СОСТОЯНИЕ СИСТЕМЫ", C.text, 1)

  local sy = y + 18
  txt(x + 6, sy, "ЭКРАН", C.dim, 1)
  txt(x + 62, sy, string.format("%dx%d  %s", W, H, short(gpuSide, 8)), C.green, 1)
  sy = sy + 12
  txt(x + 6, sy, "АППАРАТ", C.dim, 1)
  txt(x + 62, sy, subSensor and "РАБОТА" or "НЕТ", subSensor and C.green or C.red, 1)
  txt(x + 112, sy, lastSubError and short(lastSubError, 10) or "ОК", lastSubError and C.red or C.dim, 1)
  sy = sy + 12
  txt(x + 6, sy, "ИГРОКИ", C.dim, 1)
  txt(x + 62, sy, personSensor and "РАБОТА" or "НЕТ", personSensor and C.green or C.red, 1)
  txt(x + 112, sy, lastPersonError and short(lastPersonError, 10) or "ОК", lastPersonError and C.red or C.dim, 1)
  sy = sy + 12
  txt(x + 6, sy, "ДАЛЬНОСТЬ", C.dim, 1)
  txt(x + 62, sy, "А2048 / И1024", C.text, 1)
  sy = sy + 12
  txt(x + 6, sy, "СКАН", C.dim, 1)
  txt(x + 62, sy, string.format("%.2fs", SCAN_INTERVAL), C.text, 1)
  sy = sy + 12
  txt(x + 6, sy, "ЦЕЛИ", C.dim, 1)
  txt(x + 62, sy, string.format("И%d А%d", #people, #subs), C.text, 1)
  sy = sy + 12
  txt(x + 6, sy, "ВЫХОД", C.dim, 1)
  txt(x + 62, sy, physicalSiren and "СИРЕНА ВКЛ" or "СИРЕНА ВЫКЛ", physicalSiren and C.red or C.green, 1)
  sy = sy + 12
  txt(x + 6, sy, "ТИШИНА", C.dim, 1)
  local remain = math.max(0, muteUntil - now)
  txt(x + 62, sy, remain > 0 and string.format("%.0fs", remain) or "ВЫКЛ", remain > 0 and C.yellow or C.green, 1)
end

local TESTS = {
  {id="sim_player", label="ИГРОКИ", scenario="PLAYER"},
  {id="sim_fast", label="БЫСТР.", scenario="FAST"},
  {id="sim_contrap", label="АППАРАТ", scenario="CONTRAP"},
  {id="sim_inbound", label="СБЛИЖ.", scenario="INBOUND"},
  {id="sim_combined", label="ВМЕСТЕ", scenario="COMBINED"},
  {id="sim_crit", label="КРИТИЧ.", scenario="CRIT"},
}

local function drawTest(subs, people, status, message, color, bg, shouldSiren, physicalSiren, severity, focus, now)
  drawHeader(status, message, color, bg, shouldSiren, physicalSiren, severity, focus)
  local x, y, w, h = 6, 46, W - 12, H - 82
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)
  txt(x + 6, y + 5, "УПРАВЛЕНИЕ СИМУЛЯЦИЕЙ", C.text, 1)
  txt(x + 6, y + 16, simScenario and ("АКТИВНО: " .. simScenario) or "РЕАЛЬНЫЕ ДАННЫЕ", simScenario and C.yellow or C.green, 1)

  local bw, bh = 52, 22
  local startX, startY = x + 7, y + 34
  for i, t in ipairs(TESTS) do
    local col = (i - 1) % 3
    local row = math.floor((i - 1) / 3)
    local bx = startX + col * 57
    local by = startY + row * 28
    drawButton(t.id, bx, by, bw, bh, t.label, simScenario == t.scenario, C.yellow)
  end

  drawButton("sim_stop", startX, startY + 62, 80, 18, "СТОП СИМ", simScenario == nil, C.green)
  drawButton("sim_clear", startX + 86, startY + 62, 80, 18, "НЕТ ЦЕЛЕЙ", simScenario == "NONE", C.cyan)

  txt(x + 7, y + 101, QUIET_TEST and "ТИХИЙ ТЕСТ: СИРЕНА ОТКЛ." or "ТЕСТ МОЖЕТ ВКЛЮЧИТЬ СИРЕНУ", QUIET_TEST and C.yellow or C.dim, 1)
end

local function drawFooter(status, physicalSiren, now)
  local x, y, w, h = 6, H - 30, W - 12, 24
  rect(x, y, w, h, C.panel2)
  outline(x, y, w, h, C.border)

  local labels = {
    {id="filter_all", label="ВСЕ", active=filter=="all"},
    {id="filter_people", label="ИГР", active=filter=="people"},
    {id="filter_subs", label="АПП", active=filter=="subs"},
    {id="range", label=mapRange==CONTRAPTION_RANGE and "2K" or "1K", active=mapRange==PERSON_RANGE},
    {id="sys", label="СИС", active=page=="sys"},
    {id="test", label="ТСТ", active=page=="test" or simScenario~=nil},
    {id="ack", label="ОК", active=ackStatus==status},
    {id="mute", label="ТИХ", active=muteUntil>now},
  }

  local gap = 2
  local bw = math.floor((w - 4 - gap * (#labels - 1)) / #labels)
  local bx = x + 2
  for _, b in ipairs(labels) do
    local col = b.id == "mute" and C.yellow or C.cyan
    drawButton(b.id, bx, y + 3, bw, 14, b.label, b.active, col)
    bx = bx + bw + gap
  end

  if notice and noticeUntil > now then
    txtCentered(y + 17, notice, C.yellow, 1, x, w)
  else
    local sir = physicalSiren and "СИРЕНА ВКЛ" or (muteUntil > now and "СИРЕНА ТИХО" or "СИРЕНА ВЫКЛ")
    txt(x + 4, y + 17, sir, physicalSiren and C.red or (muteUntil > now and C.yellow or C.dim), 1)
    txt(x + w - 47, y + 17, textutils.formatTime(os.time(), true), C.text, 1)
  end
end

local currentStatus = "ЧИСТО"
local currentSeverity = 0
local currentShouldSiren = false
local currentPhysicalSiren = false
local currentFocus = nil

local function currentData(now)
  if simScenario ~= nil then return simData(now) end
  return liveSubs, livePeople
end

local function assignDisplayIndices(subs, people)
  for i, p in ipairs(people) do p.displayIndex = i end
  for i, c in ipairs(subs) do c.displayIndex = i end
end

local function updateAlarm(now, status, shouldSiren, severity)
  if ackStatus and ackStatus ~= status then ackStatus = nil end

  if muteUntil > now and severity > muteSeverity then
    muteUntil = 0
    muteSeverity = 0
    setNotice("ТИШИНА СНЯТА: НОВАЯ УГРОЗА", 3)
  end

  local muted = muteUntil > now
  local physical = shouldSiren and not muted
  if simScenario and QUIET_TEST then physical = false end
  rs.setOutput(SIREN_SIDE, physical)
  return physical
end

local function render()
  local now = nowSeconds()
  hitTargets = {}

  local subs, people = currentData(now)
  assignDisplayIndices(subs, people)
  local status, message, color, bg, shouldSiren, severity, focus = summarize(subs, people)
  local physicalSiren = updateAlarm(now, status, shouldSiren, severity)

  currentStatus = status
  currentSeverity = severity
  currentShouldSiren = shouldSiren
  currentPhysicalSiren = physicalSiren
  currentFocus = focus

  gpu.fill(C.bg)
  if page == "sys" then
    drawSystem(subs, people, status, message, color, bg, shouldSiren, physicalSiren, severity, focus, now)
  elseif page == "test" then
    drawTest(subs, people, status, message, color, bg, shouldSiren, physicalSiren, severity, focus, now)
  else
    drawMain(subs, people, status, message, color, bg, shouldSiren, physicalSiren, severity, focus, now)
  end
  drawFooter(status, physicalSiren, now)
  gpu.sync()
end

local function scanLive()
  local now = nowSeconds()
  liveSubs = scanSubs(now)
  livePeople = scanPeople(now)
end

local function clickTrack(data)
  if not data then return end
  selectedKind = data.kind
  selectedKey = data.key
  page = "main"
end

local function handleClick(x, y, sneak)
  local now = nowSeconds()
  for i = #hitTargets, 1, -1 do
    local h = hitTargets[i]
    if x >= h.x and x < h.x + h.w and y >= h.y and y < h.y + h.h then
      local id = h.id
      if id == "track" then
        clickTrack(h.data)
      elseif id == "header" then
        clickTrack(h.data)
      elseif id == "back" then
        selectedKind, selectedKey = nil, nil
      elseif id == "list_up" then
        listOffset = math.max(1, listOffset - LIST_ROWS)
      elseif id == "list_down" then
        listOffset = listOffset + LIST_ROWS
      elseif id == "filter_all" then
        filter, listOffset, page = "all", 1, "main"
      elseif id == "filter_people" then
        filter, listOffset, page = "people", 1, "main"
      elseif id == "filter_subs" then
        filter, listOffset, page = "subs", 1, "main"
      elseif id == "range" then
        mapRange = mapRange == CONTRAPTION_RANGE and PERSON_RANGE or CONTRAPTION_RANGE
        page = "main"
      elseif id == "sys" then
        page = page == "sys" and "main" or "sys"
      elseif id == "test" then
        page = page == "test" and "main" or "test"
      elseif id == "ack" then
        ackStatus = currentStatus
        setNotice("ТРЕВОГА ПРИНЯТА", 2)
      elseif id == "mute" then
        if not sneak then
          setNotice("SHIFT + ТИХ ДЛЯ ОТКЛ. ЗВУКА", 3)
        elseif currentShouldSiren then
          muteUntil = now + MUTE_SECONDS
          muteSeverity = currentSeverity
          setNotice("СИРЕНА ОТКЛ. НА " .. tostring(MUTE_SECONDS) .. "с", 3)
        else
          muteUntil = 0
          muteSeverity = 0
          setNotice("СИРЕНА НЕ АКТИВНА", 2)
        end
      elseif id == "sim_stop" then
        simScenario = nil
        muteUntil, muteSeverity = 0, 0
        page = "main"
        selectedKind, selectedKey = nil, nil
        setNotice("СИМУЛЯЦИЯ ОСТАНОВЛЕНА", 2)
      elseif id == "sim_clear" then
        simScenario = "NONE"
        simStart = now
        setNotice("СИМ: ЧИСТО", 2)
      elseif id == "sim_player" then
        simScenario, simStart = "PLAYER", now
      elseif id == "sim_fast" then
        simScenario, simStart = "FAST", now
      elseif id == "sim_contrap" then
        simScenario, simStart = "CONTRAP", now
      elseif id == "sim_inbound" then
        simScenario, simStart = "INBOUND", now
      elseif id == "sim_combined" then
        simScenario, simStart = "COMBINED", now
      elseif id == "sim_crit" then
        simScenario, simStart = "CRIT", now
      end
      render()
      return
    end
  end
end

local function main()
  rs.setOutput(SIREN_SIDE, false)
  scanLive()
  render()

  local scanTimer = os.startTimer(SCAN_INTERVAL)
  local frameTimer = os.startTimer(FRAME_INTERVAL)

  while true do
    local ev, a, b, c, d = os.pullEvent()
    if ev == "tm_monitor_touch" then
      -- event, gpuSide, x, y, sneak
      handleClick(b, c, d)
    elseif ev == "timer" then
      if a == scanTimer then
        scanLive()
        scanTimer = os.startTimer(SCAN_INTERVAL)
        render()
      elseif a == frameTimer then
        frameTimer = os.startTimer(FRAME_INTERVAL)
        render()
      end
    end
  end
end

local ok, err = pcall(main)
rs.setOutput(SIREN_SIDE, false)
if not ok then error(err, 0) end
