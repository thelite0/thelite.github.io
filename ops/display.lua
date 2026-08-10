local gpu, gpuSide
for _, name in ipairs(peripheral.getNames()) do
  local p = peripheral.wrap(name)
  if p and type(p.refreshSize) == "function" and type(p.sync) == "function" and type(p.getSize) == "function" then
    gpu, gpuSide = p, name
    break
  end
end
assert(gpu, "compatible display controller not found")

print("display controller: " .. tostring(gpuSide))

-- refreshSize() is asynchronous in Tom's Peripherals. Do not call setSize()
-- until the GPU has actually discovered at least one attached monitor.
local W, H = 0, 0
for i = 1, 20 do
  gpu.refreshSize()
  sleep(0.1)
  W, H = gpu.getSize()
  if type(W) == "number" and type(H) == "number" and W > 0 and H > 0 then break end
end

assert(type(W) == "number" and type(H) == "number" and W > 0 and H > 0,
  "no bitmap display detected by GPU")

gpu.setSize(64)
sleep(0.1)
W, H = gpu.getSize()
print("display size: " .. tostring(W) .. "x" .. tostring(H))

local C = {
  bg     = 0xFF071016,
  panel  = 0xFF0D1921,
  panel2 = 0xFF101F29,
  grid   = 0xFF173541,
  border = 0xFF245266,
  text   = 0xFFD7EDF5,
  dim    = 0xFF6E8791,
  cyan   = 0xFF42D5F5,
  green  = 0xFF51E59A,
  yellow = 0xFFF3CC5C,
  red    = 0xFFFF5D68,
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
  local maxW = MAX_X - x
  local maxH = MAX_Y - y
  w = math.min(w, maxW)
  h = math.min(h, maxH)
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
  local ok = pcall(gpu.drawText, x, y, tostring(s), c or C.text, 0x00000000, size or 1)
  return ok
end

local function circle(cx, cy, r, c, segments)
  segments = segments or 36
  local px, py = cx + r, cy
  for i = 1, segments do
    local a = (i / segments) * math.pi * 2
    local x = cx + math.cos(a) * r
    local y = cy + math.sin(a) * r
    line(px, py, x, y, c)
    px, py = x, y
  end
end

local contacts = {
  {x=.62, y=.24, level=2},
  {x=-.30, y=-.55, level=1},
  {x=.18, y=-.15, level=3},
}
local levelColor = {C.green, C.yellow, C.red}
local sweep = 0

while true do
  gpu.fill(C.bg)

  rect(3, 3, W - 7, 11, C.panel2)
  rect(3, 13, W - 7, 1, C.cyan)
  txt(6, 5, "NODE", C.text, 1)
  if W >= 58 then txt(W - 42, 5, "ONLINE", C.green, 1) end

  local x, y = 3, 17
  local sw, sh = W - 7, H - 21
  rect(x, y, sw, sh, C.panel)
  outline(x, y, sw, sh, C.border)

  local cx = math.floor(W / 2)
  local cy = y + math.floor(sh / 2)
  local r = math.max(5, math.floor(math.min(sw, sh) * 0.39))
  r = math.min(r, cx - MIN_X - 2, MAX_X - cx - 2, cy - MIN_Y - 2, MAX_Y - cy - 2)

  if r >= 4 then
    circle(cx, cy, r, C.border)
    circle(cx, cy, math.floor(r * .66), C.grid)
    circle(cx, cy, math.floor(r * .33), C.grid)
    line(cx-r, cy, cx+r, cy, C.grid)
    line(cx, cy-r, cx, cy+r, C.grid)

    local sx = cx + math.cos(sweep) * r
    local sy = cy + math.sin(sweep) * r
    line(cx, cy, sx, sy, C.cyan)

    for d = 1, 3 do
      local a = sweep - d * .055
      line(cx, cy, cx + math.cos(a) * r, cy + math.sin(a) * r, 0xFF123847)
    end

    rect(cx-1, cy-1, 3, 3, C.cyan)

    for _, contact in ipairs(contacts) do
      local px = cx + contact.x * r
      local py = cy + contact.y * r
      rect(px-2, py-2, 5, 5, levelColor[contact.level])
    end
  end

  gpu.sync()
  sweep = (sweep + .055) % (math.pi * 2)
  sleep(.05)
end
