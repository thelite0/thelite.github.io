local args = {...}
local TEST_MODE = args[1] == "test"
local QUIET_TEST = args[2] == "quiet"

local RANGE = 2048
local SIREN_SIDE = "back"
local SCAN_INTERVAL = 1.0
local HISTORY = 8
local TRACK_TTL = 12

-- Known monitored sector in NeoRadar local coordinates (radar - target).
local SECTOR_X, SECTOR_Z = 1881, 1632
local SECTOR_LEN = math.sqrt(SECTOR_X * SECTOR_X + SECTOR_Z * SECTOR_Z)
local SECTOR_COS = 0.70

local function nowSeconds()
  return os.epoch("utc") / 1000
end

-- Find Tom's GPU without depending on which side it is attached to.
local gpu, gpuSide
for _, name in ipairs(peripheral.getNames()) do
  local p = peripheral.wrap(name)
  if p and type(p.refreshSize) == "function" and type(p.sync) == "function" and type(p.getSize) == "function" then
    gpu, gpuSide = p, name
    break
  end
end
assert(gpu, "compatible display controller not found")

-- Tom's refreshSize() is asynchronous. Wait for a real monitor size.
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

-- Find NeoRadar.
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
  bg      = 0xFF05090C,
  panel   = 0xFF0A1217,
  panel2  = 0xFF0D1A20,
  grid    = 0xFF16323A,
  border  = 0xFF2B5560,
  text    = 0xFFE5F0F2,
  dim     = 0xFF71858C,
  cyan    = 0xFF49D9F2,
  green   = 0xFF51E39B,
  yellow  = 0xFFF2D35F,
  orange  = 0xFFF39A4A,
  red     = 0xFFFF5964,
  darkred = 0xFF351218,
}

-- The GPU is strict about primitives touching the framebuffer edge.
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
  y = clamp(math.floor(y), MIN_Y, MAX_Y - 7)
  pcall(gpu.drawText, x, y, tostring(s), c or C.text, 0x00000000, size or 1)
end

local function uuidKey(v)
  local q = v.id or {}
  return tostring(q[1]) .. ":" .. tostring(q[2]) .. ":" .. tostring(q[3]) .. ":" .. tostring(q[4])
end

local function inSector(x, z)
  local hz = math.sqrt(x * x + z * z)
  if hz <= 0 then return false end
  return (x * SECTOR_X + z * SECTOR_Z) / (hz * SECTOR_LEN) > SECTOR_COS
end

-- Linear regression of distance over time. Positive result means closing.
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
  local slope = (n * std - st * sd) / den
  return -slope
end

local function totalSpeed(h)
  if #h < 2 then return 0 end
  local a, b = h[1], h[#h]
  local dt = b.t - a.t
  if dt <= 0 then return 0 end
  local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
  return math.sqrt(dx * dx + dy * dy + dz * dz) / dt
end

local tracks = {}
local rows = {}

local function classify(row, samples)
  local rank, label = 0, "TRACK"
  if row.sector then rank, label = 1, "WATCH" end

  local threshold = row.d > 1500 and 12 or 6
  local inbound = row.sector and samples >= 4 and row.closing > threshold
  if inbound then
    rank, label = 2, "INBOUND"
    if row.d < 1200 or (row.eta and row.eta < 60) then rank, label = 3, "HIGH" end
    if row.d < 500 or (row.eta and row.eta < 20) then rank, label = 4, "CRIT" end
  end

  row.rank, row.label = rank, label
end

local function scanReal(now)
  local result = radar.scanForSubLevels(RANGE) or {}
  local out = {}

  for _, v in ipairs(result) do
    local k = uuidKey(v)
    local tr = tracks[k]
    if not tr then
      tr = {history = {}, firstSeen = now}
      tracks[k] = tr
    end
    tr.lastSeen = now

    local h = tr.history
    h[#h + 1] = {t = now, d = v.distance, x = v.x, y = v.y, z = v.z}
    while #h > HISTORY do table.remove(h, 1) end

    local cl = closingRate(h)
    local sp = totalSpeed(h)
    local eta = cl > 1 and (v.distance / cl) or nil

    local row = {
      key = k,
      name = v.name or "UNKNOWN",
      x = v.x,
      y = v.y,
      z = v.z,
      d = v.distance,
      closing = cl,
      speed = sp,
      eta = eta,
      sector = inSector(v.x, v.z),
      samples = #h,
    }
    classify(row, #h)
    out[#out + 1] = row
  end

  for k, tr in pairs(tracks) do
    if not tr.lastSeen or now - tr.lastSeen > TRACK_TTL then tracks[k] = nil end
  end

  table.sort(out, function(a, b)
    if a.rank ~= b.rank then return a.rank > b.rank end
    return a.d < b.d
  end)
  return out
end

local function scanTest(now)
  local phase = now % 24
  if phase >= 18 then return {} end

  local label, rank, d, closing, speed
  if phase < 4 then
    label, rank, d, closing, speed = "WATCH", 1, 1900, 0, 0
  elseif phase < 9 then
    label, rank = "INBOUND", 2
    d = 1900 - (phase - 4) * 80
    closing, speed = 80, 82
  elseif phase < 14 then
    label, rank = "HIGH", 3
    d = 1200 - (phase - 9) * 110
    closing, speed = 110, 112
  else
    label, rank = "CRIT", 4
    d = math.max(120, 480 - (phase - 14) * 75)
    closing, speed = 75, 78
  end

  local dx = SECTOR_X / SECTOR_LEN
  local dz = SECTOR_Z / SECTOR_LEN
  local eta = closing > 1 and d / closing or nil
  return {{
    key = "TEST",
    name = "SIM",
    x = dx * d,
    y = 0,
    z = dz * d,
    d = d,
    closing = closing,
    speed = speed,
    eta = eta,
    sector = true,
    samples = 8,
    rank = rank,
    label = label,
  }}
end

local function overallStatus(current)
  if #current == 0 then return 0, "CLEAR", C.green end
  local top = current[1]
  if top.rank >= 4 then return 4, "CRIT", C.red end
  if top.rank == 3 then return 3, "HIGH", C.red end
  if top.rank == 2 then return 2, "INBOUND", C.orange end
  if top.rank == 1 then return 1, "WATCH", C.yellow end
  return 0, "CLEAR", C.green
end

local function setSiren(on)
  if TEST_MODE and QUIET_TEST then on = false end
  rs.setOutput(SIREN_SIDE, on)
end

local function draw(current, alarm)
  local rank, status, statusColor = overallStatus(current)
  local top = current[1]

  gpu.fill(C.bg)

  -- Explicit system state. No fake rotating sweep: NeoRadar is a static volume scan.
  rect(2, 2, W - 4, 11, rank >= 3 and C.darkred or C.panel2)
  txt(5, 4, status, statusColor, 1)
  txt(39, 4, alarm and "A:ON" or "A:OFF", alarm and C.red or C.dim, 1)
  if TEST_MODE then txt(24, 4, "TEST", C.cyan, 1) end
  rect(2, 12, W - 4, 1, statusColor)

  -- Static top-down coverage map. The square is truthful: NeoRadar scans an AABB.
  local mx, my, mw, mh = 2, 16, 32, 32
  rect(mx, my, mw, mh, C.panel)
  outline(mx, my, mw, mh, C.border)

  local cx = mx + math.floor(mw / 2)
  local cy = my + math.floor(mh / 2)
  line(cx, my + 1, cx, my + mh - 2, C.grid)
  line(mx + 1, cy, mx + mw - 2, cy, C.grid)
  line(mx + 8, my + 1, mx + 8, my + mh - 2, C.grid)
  line(mx + 24, my + 1, mx + 24, my + mh - 2, C.grid)
  line(mx + 1, my + 8, mx + mw - 2, my + 8, C.grid)
  line(mx + 1, my + 24, mx + mw - 2, my + 24, C.grid)

  -- Own position.
  rect(cx - 1, cy - 1, 3, 3, C.cyan)

  for _, c in ipairs(current) do
    -- Raw NeoRadar x/z are mapped directly into the +/- RANGE square.
    local px = cx + (c.x / RANGE) * (mw / 2 - 2)
    local py = cy + (c.z / RANGE) * (mh / 2 - 2)
    local col = C.green
    if c.rank == 1 then col = C.yellow end
    if c.rank == 2 then col = C.orange end
    if c.rank >= 3 then col = C.red end
    rect(px - 1, py - 1, 3, 3, col)
    if c.rank >= 3 then outline(px - 2, py - 2, 5, 5, col) end
  end

  -- Compact 64x64 data panel.
  local sx = 37
  txt(sx, 17, "CONTACT", C.dim, 1)
  txt(sx, 24, "C " .. tostring(#current), C.text, 1)
  if top then
    txt(sx, 31, "D " .. tostring(math.floor(top.d + 0.5)), C.text, 1)
    txt(sx, 38, string.format("V %+.0f", top.closing), top.closing > 1 and C.orange or C.dim, 1)
    local eta = top.eta and (tostring(math.floor(top.eta + 0.5)) .. "s") or "--"
    txt(sx, 45, "E " .. eta, C.text, 1)
  else
    txt(sx, 31, "D --", C.dim, 1)
    txt(sx, 38, "V --", C.dim, 1)
    txt(sx, 45, "E --", C.dim, 1)
  end

  txt(3, 52, "R2048", C.dim, 1)
  txt(27, 52, radar and "SENSOR OK" or "SIM DATA", radar and C.green or C.cyan, 1)
  txt(3, 59, alarm and "SIREN ON" or "SIREN OFF", alarm and C.red or C.dim, 1)

  gpu.sync()
end

local lastScan = -100
local function main()
  rs.setOutput(SIREN_SIDE, false)
  while true do
    local now = nowSeconds()
    if now - lastScan >= SCAN_INTERVAL then
      rows = TEST_MODE and scanTest(now) or scanReal(now)
      lastScan = now
    end

    local alarm = (#rows > 0 and rows[1].rank >= 2)
    setSiren(alarm)
    draw(rows, alarm and not (TEST_MODE and QUIET_TEST))
    sleep(0.15)
  end
end

local ok, err = pcall(main)
rs.setOutput(SIREN_SIDE, false)
if not ok then error(err, 0) end
