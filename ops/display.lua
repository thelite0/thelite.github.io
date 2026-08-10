local args = {...}
local TEST_MODE = args[1] == "test"
local QUIET_TEST = args[2] == "quiet"

local RANGE = 2048
local SIREN_SIDE = "back"
local SCAN_INTERVAL = 1.0
local HISTORY = 8
local TRACK_TTL = 12

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
  bg        = 0xFF070B0D,
  panel     = 0xFF10161A,
  panel2    = 0xFF131C21,
  panel3    = 0xFF0B1115,
  border    = 0xFF38505B,
  grid      = 0xFF17313A,
  stripeA   = 0xFF644F17,
  stripeB   = 0xFF1A1710,
  text      = 0xFFE5EEF2,
  dim       = 0xFF7C8F97,
  cyan      = 0xFF4BDCF6,
  green     = 0xFF4FE39B,
  yellow    = 0xFFF0D15A,
  orange    = 0xFFFF9D3A,
  red       = 0xFFFF5C68,
  magenta   = 0xFFFF62D3,
  darkGreen = 0xFF19392A,
  darkRed   = 0xFF38191D,
}

local LEVELS = {
  TRACK   = {rank = 0, color = C.cyan},
  WATCH   = {rank = 1, color = C.yellow},
  INBOUND = {rank = 2, color = C.orange},
  HIGH    = {rank = 3, color = C.red},
  CRIT    = {rank = 4, color = C.magenta},
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
  local maxW = MAX_X - x + 1
  local maxH = MAX_Y - y + 1
  w = math.min(w, maxW)
  h = math.min(h, maxH)
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
  local ok = pcall(gpu.drawText, x, y, tostring(s), c or C.text, 0x00000000, size or 1)
  if not ok and (size or 1) ~= 1 then
    pcall(gpu.drawText, x, y, tostring(s), c or C.text, 0x00000000, 1)
  end
end

local function stripes(x, y, w, h)
  rect(x, y, w, h, C.panel3)
  for i = -h, w, 10 do
    line(x + i, y + h - 1, x + i + h, y, C.stripeA)
    line(x + i + 4, y + h - 1, x + i + h + 4, y, C.stripeB)
  end
  outline(x, y, w, h, C.border)
end

local function bar(x, y, w, h, frac, fill, bg)
  rect(x, y, w, h, bg or C.panel3)
  outline(x, y, w, h, C.border)
  local fw = math.max(0, math.min(w - 2, math.floor((w - 2) * clamp(frac, 0, 1))))
  if fw > 0 then rect(x + 1, y + 1, fw, h - 2, fill) end
end

local function shortLabel(s, n)
  s = tostring(s or "UNKNOWN")
  if #s <= n then return s end
  return s:sub(1, math.max(1, n - 3)) .. "..."
end

local function keyOf(v)
  if type(v.id) == "table" then
    local out = {}
    for _, k in ipairs(v.id) do out[#out + 1] = tostring(k) end
    if #out > 0 then return table.concat(out, ":") end
  elseif v.id ~= nil then
    return tostring(v.id)
  end
  local name = v.name and tostring(v.name) or "CONTACT"
  return name .. "@" .. math.floor(v.x or 0) .. ":" .. math.floor(v.z or 0)
end

local function inSector(x, z)
  local h = math.sqrt(x * x + z * z)
  if h < 1 then return false, 0 end
  local dot = (x * SECTOR_X + z * SECTOR_Z) / (h * SECTOR_LEN)
  return dot >= SECTOR_COS, dot
end

local tracks = {}
local logs = {}
local lastState = "BOOT"
local lastSiren = false
local lastCount = -1

local function pushLog(msg)
  table.insert(logs, 1, string.format("[%s] %s", textutils.formatTime(os.time(), true), msg))
  while #logs > 4 do table.remove(logs) end
end
pushLog(TEST_MODE and "TEST MODE ACTIVE" or "DISPLAY ONLINE")

local function ensureTrack(k)
  local t = tracks[k]
  if not t then
    t = {history = {}, name = k}
    tracks[k] = t
  end
  return t
end

local function updateHistory(track, sample, now)
  local h = track.history
  h[#h + 1] = {t = now, d = sample.distance or 0, x = sample.x or 0, y = sample.y or 0, z = sample.z or 0}
  while #h > HISTORY do table.remove(h, 1) end
  track.lastSeen = now
  track.name = sample.name or track.name
end

local function metrics(track)
  local h = track.history
  local spd, closing = 0, 0
  if #h >= 2 then
    local a, b = h[1], h[#h]
    local dt = b.t - a.t
    if dt > 0 then
      local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
      spd = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
      closing = (a.d - b.d) / dt
    end
  end
  return spd, closing
end

local function classify(track, sample)
  local d = sample.distance or 0
  local speed, closing = metrics(track)
  local sector, dot = inSector(sample.x or 0, sample.z or 0)
  local eta = closing > 0.5 and d / closing or nil

  local state = "TRACK"
  local rank = 0

  if sector then
    state, rank = "WATCH", 1
    local inbound = closing > (d > 1500 and 10 or d > 900 and 6 or 3)
    if inbound then
      state, rank = "INBOUND", 2
      if d < 1200 or (eta and eta < 70) then state, rank = "HIGH", 3 end
      if d < 550 or (eta and eta < 25) then state, rank = "CRIT", 4 end
    end
  end

  local level = LEVELS[state]
  return {
    key = keyOf(sample),
    name = sample.name or track.name or "UNKNOWN",
    x = sample.x or 0,
    y = sample.y or 0,
    z = sample.z or 0,
    d = d,
    speed = speed,
    closing = closing,
    eta = eta,
    sector = sector,
    dot = dot,
    state = state,
    rank = rank,
    color = level.color,
  }
end

local function fakeContacts(now)
  local t = now % 44
  local list = {}

  local scout = {id = "ghost-scout", name = "SCOUT", x = -680, y = 20, z = -920}
  scout.distance = math.sqrt(scout.x * scout.x + scout.y * scout.y + scout.z * scout.z)
  list[#list + 1] = scout

  if t < 6 then
    return list
  elseif t < 14 then
    local x, z = 1780, 1540
    list[#list + 1] = {id = "hostile-1", name = "BOMBER", x = x, y = 36, z = z, distance = math.sqrt(x*x + 36*36 + z*z)}
  elseif t < 24 then
    local p = (t - 14) / 10
    local x = 1780 - p * 780
    local z = 1540 - p * 680
    list[#list + 1] = {id = "hostile-1", name = "BOMBER", x = x, y = 35, z = z, distance = math.sqrt(x*x + 35*35 + z*z)}
  elseif t < 32 then
    local p = (t - 24) / 8
    local x = 1000 - p * 520
    local z = 860 - p * 460
    list[#list + 1] = {id = "hostile-1", name = "BOMBER", x = x, y = 28, z = z, distance = math.sqrt(x*x + 28*28 + z*z)}
  elseif t < 38 then
    local p = (t - 32) / 6
    local x = 480 - p * 240
    local z = 400 - p * 200
    list[#list + 1] = {id = "hostile-1", name = "BOMBER", x = x, y = 22, z = z, distance = math.sqrt(x*x + 22*22 + z*z)}
  end

  return list
end

local function collect(now)
  local raw = TEST_MODE and fakeContacts(now) or radar.scanForSubLevels(RANGE)
  local seen = {}
  local out = {}

  for _, sample in ipairs(raw or {}) do
    sample.distance = sample.distance or math.sqrt((sample.x or 0)^2 + (sample.y or 0)^2 + (sample.z or 0)^2)
    local k = keyOf(sample)
    seen[k] = true
    local t = ensureTrack(k)
    updateHistory(t, sample, now)
    out[#out + 1] = classify(t, sample)
  end

  for k, t in pairs(tracks) do
    if not seen[k] and t.lastSeen and now - t.lastSeen > TRACK_TTL then
      tracks[k] = nil
    end
  end

  table.sort(out, function(a, b)
    if a.rank ~= b.rank then return a.rank > b.rank end
    return a.d < b.d
  end)

  local top = out[1]
  local state = top and top.state or "CLEAR"
  local siren = top and top.rank >= 2 or false
  if TEST_MODE and QUIET_TEST then siren = false end

  if state ~= lastState then
    pushLog("STATE " .. state)
    lastState = state
  end
  if #out ~= lastCount then
    pushLog("CONTACTS " .. tostring(#out))
    lastCount = #out
  end
  if siren ~= lastSiren then
    pushLog(siren and "SIREN ARMED" or "SIREN CLEAR")
    lastSiren = siren
  end

  return out, top, state, siren
end

local function drawRadarBox(x, y, size, contacts, top)
  rect(x, y, size, size, C.panel)
  outline(x, y, size, size, C.border)

  local cx = x + math.floor(size / 2)
  local cy = y + math.floor(size / 2)
  local half = math.floor((size - 8) / 2)

  for i = 1, 3 do
    local o = math.floor(half * i / 3)
    outline(cx - o, cy - o, o * 2, o * 2, C.grid)
  end
  line(cx - half, cy, cx + half, cy, C.grid)
  line(cx, cy - half, cx, cy + half, C.grid)

  local ang = math.atan(SECTOR_Z, SECTOR_X)
  local spread = math.acos(SECTOR_COS)
  local l1x = cx + math.cos(ang - spread) * half
  local l1y = cy + math.sin(ang - spread) * half
  local l2x = cx + math.cos(ang + spread) * half
  local l2y = cy + math.sin(ang + spread) * half
  line(cx, cy, l1x, l1y, C.darkGreen)
  line(cx, cy, l2x, l2y, C.darkGreen)
  txt(cx + math.cos(ang) * (half - 16), cy + math.sin(ang) * (half - 16), "RIVAL", C.yellow, 1)

  rect(cx - 2, cy - 2, 4, 4, C.cyan)

  for _, c in ipairs(contacts) do
    local px = cx + math.floor((c.x / RANGE) * half)
    local py = cy + math.floor((c.z / RANGE) * half)
    local s = c == top and 6 or 4
    rect(px - math.floor(s/2), py - math.floor(s/2), s, s, c.color)
    if c == top then
      outline(px - math.floor(s/2) - 2, py - math.floor(s/2) - 2, s + 4, s + 4, c.color)
    end
  end

  txt(x + 5, y + 5, "SCAN FIELD", C.dim, 1)
  txt(x + size - 34, y + size - 10, tostring(RANGE), C.dim, 1)
end

local function drawFrame(state, contacts, top, siren)
  local stateInfo = LEVELS[state] or {color = C.green}
  if state == "CLEAR" then stateInfo = {color = C.green} end

  gpu.fill(C.bg)

  local margin = 6
  local headerH = 22
  local footerH = 34
  local sideW = 62
  local gap = 6
  local radarSize = math.min(H - headerH - footerH - margin * 2 - gap, W - sideW - margin * 2 - gap)
  radarSize = math.max(96, radarSize)
  local radarX = margin
  local radarY = headerH + margin
  local sideX = radarX + radarSize + gap
  local sideY = radarY
  local footerY = radarY + radarSize + gap

  rect(margin, margin, W - margin * 2, headerH, C.panel2)
  outline(margin, margin, W - margin * 2, headerH, C.border)
  stripes(margin + 2, margin + 2, 42, headerH - 4)
  stripes(W - margin - 44, margin + 2, 42, headerH - 4)
  txt(margin + 48, margin + 4, "CRYSTALITE EARLY WARNING", C.text, 1)
  txt(margin + 48, margin + 12, TEST_MODE and (QUIET_TEST and "TEST / QUIET" or "TEST / LIVE") or "LIVE SENSOR LINK", C.dim, 1)

  local pillW = 54
  rect(sideX, margin + 4, pillW, 14, state == "CLEAR" and C.darkGreen or C.darkRed)
  outline(sideX, margin + 4, pillW, 14, stateInfo.color)
  txt(sideX + 6, margin + 7, state, stateInfo.color, 1)
  txt(sideX + pillW + 8, margin + 7, siren and "A:ON" or "A:OFF", siren and C.red or C.green, 1)

  drawRadarBox(radarX, radarY, radarSize, contacts, top)

  rect(sideX, sideY, sideW, 56, C.panel)
  outline(sideX, sideY, sideW, 56, C.border)
  txt(sideX + 5, sideY + 4, "PRIMARY THREAT", C.text, 1)
  if top then
    txt(sideX + 5, sideY + 16, shortLabel(top.name, 10), top.color, 1)
    txt(sideX + 5, sideY + 28, string.format("DIST %4.0f", top.d), C.text, 1)
    txt(sideX + 5, sideY + 38, string.format("CLS  %+.1f/s", top.closing), top.closing > 0 and C.orange or C.cyan, 1)
    txt(sideX + 5, sideY + 48, top.eta and string.format("ETA  %2.0fs", top.eta) or "ETA  --", C.text, 1)
  else
    txt(sideX + 5, sideY + 20, "NO TRACKS", C.dim, 1)
    txt(sideX + 5, sideY + 32, "AIRSPACE CLEAR", C.green, 1)
  end

  local sysY = sideY + 62
  rect(sideX, sysY, sideW, 38, C.panel)
  outline(sideX, sysY, sideW, 38, C.border)
  txt(sideX + 5, sysY + 4, "SYSTEM", C.text, 1)
  txt(sideX + 5, sysY + 16, "RADAR " .. (TEST_MODE and "SIM" or "OK"), TEST_MODE and C.yellow or C.green, 1)
  txt(sideX + 5, sysY + 26, string.format("DSP %dx%d", W, H), C.dim, 1)
  txt(sideX + 34, sysY + 26, shortLabel(gpuSide, 6), C.dim, 1)

  local listY = sysY + 44
  local listH = radarY + radarSize - listY
  rect(sideX, listY, sideW, listH, C.panel)
  outline(sideX, listY, sideW, listH, C.border)
  txt(sideX + 5, listY + 4, "TRACK LIST", C.text, 1)
  for i = 1, math.min(6, #contacts) do
    local c = contacts[i]
    local yy = listY + 10 + (i - 1) * 14
    rect(sideX + 4, yy, 6, 6, c.color)
    txt(sideX + 14, yy - 1, shortLabel(c.name, 7), C.text, 1)
    txt(sideX + sideW - 22, yy - 1, string.format("%3.0f", c.d), C.dim, 1)
  end

  rect(margin, footerY, W - margin * 2, footerH, C.panel2)
  outline(margin, footerY, W - margin * 2, footerH, C.border)
  stripes(margin + 4, footerY + 4, 28, footerH - 8)
  txt(margin + 36, footerY + 5, siren and "SIREN ACTIVE" or "SIREN STANDBY", siren and C.red or C.green, 1)
  txt(margin + 36, footerY + 15, string.format("CONTACTS %d   RANGE %d", #contacts, RANGE), C.dim, 1)
  txt(W - 70, footerY + 5, textutils.formatTime(os.time(), true), C.text, 1)
  txt(W - 70, footerY + 15, TEST_MODE and "SIMULATED" or "LIVE", TEST_MODE and C.yellow or C.cyan, 1)
  bar(W - 70, footerY + 22, 56, 8, math.min(1, #contacts / 6), stateInfo.color, C.panel3)

  for i = 1, math.min(3, #logs) do
    txt(margin + 36, footerY + 23 + (i - 1) * 9, shortLabel(logs[i] or "", 40), C.dim, 1)
  end

  gpu.sync()
end

while true do
  local now = os.epoch("utc") / 1000
  local contacts, top, state, siren = collect(now)
  rs.setOutput(SIREN_SIDE, siren)
  drawFrame(state, contacts, top, siren)
  sleep(SCAN_INTERVAL)
end
