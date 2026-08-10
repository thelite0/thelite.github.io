local gpu = peripheral.wrap("bottom")
assert(gpu, "display controller not found on bottom")
assert(gpu.refreshSize and gpu.sync, "bottom peripheral is not a compatible display controller")

gpu.refreshSize()
gpu.setSize(64)

local W, H = gpu.getSize()
assert(type(W) == "number" and type(H) == "number", "could not determine display size")

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

local function clamp(v, a, b)
  if v < a then return a end
  if v > b then return b end
  return v
end

local function rect(x, y, w, h, c)
  x, y, w, h = math.floor(x), math.floor(y), math.floor(w), math.floor(h)
  if w <= 0 or h <= 0 then return end
  if x < 0 then w, x = w + x, 0 end
  if y < 0 then h, y = h + y, 0 end
  if x >= W or y >= H then return end
  w = math.min(w, W - x)
  h = math.min(h, H - y)
  if w > 0 and h > 0 then gpu.filledRectangle(x, y, w, h, c) end
end

local function outline(x, y, w, h, c)
  x, y, w, h = math.floor(x), math.floor(y), math.floor(w), math.floor(h)
  if w <= 1 or h <= 1 or x < 0 or y < 0 or x + w > W or y + h > H then return end
  gpu.rectangle(x, y, w, h, c)
end

local function txt(x, y, s, c, size)
  x, y = math.floor(x), math.floor(y)
  if x < 0 or y < 0 or x >= W or y >= H then return end
  gpu.drawText(x, y, tostring(s), c or C.text, 0x00000000, size or 1)
end

local function line(x1, y1, x2, y2, c)
  x1 = clamp(math.floor(x1), 0, W - 1)
  y1 = clamp(math.floor(y1), 0, H - 1)
  x2 = clamp(math.floor(x2), 0, W - 1)
  y2 = clamp(math.floor(y2), 0, H - 1)
  gpu.lineS(x1, y1, x2, y2, c)
end

local function circle(cx, cy, r, c, segments)
  segments = segments or 40
  local px, py = cx + r, cy
  for i = 1, segments do
    local a = (i / segments) * math.pi * 2
    local x, y = cx + math.cos(a) * r, cy + math.sin(a) * r
    line(px, py, x, y, c)
    px, py = x, y
  end
end

local contacts = {
  {x=.62, y=.24, level=2, id="A17"},
  {x=-.30, y=-.55, level=1, id="C04"},
  {x=.18, y=-.15, level=3, id="B22"},
}

local levelColor = {C.green, C.yellow, C.red}
local sweep = 0

local function drawScope(x, y, w, h, compact)
  rect(x, y, w, h, C.panel)
  outline(x, y, w, h, C.border)

  local cx = x + math.floor(w / 2)
  local cy = y + math.floor(h / 2)
  local r = math.max(6, math.floor(math.min(w, h) * (compact and .40 or .42)))

  circle(cx, cy, r, C.border)
  circle(cx, cy, math.floor(r * .66), C.grid)
  circle(cx, cy, math.floor(r * .33), C.grid)
  line(cx-r, cy, cx+r, cy, C.grid)
  line(cx, cy-r, cx, cy+r, C.grid)

  local sx = cx + math.cos(sweep) * r
  local sy = cy + math.sin(sweep) * r
  line(cx, cy, sx, sy, C.cyan)
  for d = 1, 3 do
    local a = sweep - d * 0.05
    line(cx, cy, cx + math.cos(a) * r, cy + math.sin(a) * r, 0xFF123847)
  end

  rect(cx-1, cy-1, 3, 3, C.cyan)

  for _, c in ipairs(contacts) do
    local px = cx + c.x * r
    local py = cy + c.y * r
    local col = levelColor[clamp(c.level, 1, 3)]
    rect(px-2, py-2, 5, 5, col)
    if not compact then
      outline(px-4, py-4, 9, 9, col)
      txt(px+6, py-3, c.id, col, 1)
    end
  end
end

while true do
  gpu.fill(C.bg)

  if W < 120 or H < 80 then
    -- Compact layout for a single/small panel.
    rect(0, 0, W, 11, C.panel2)
    rect(0, 10, W, 1, C.cyan)
    txt(3, 2, "NODE", C.text, 1)
    txt(math.max(30, W - 27), 2, "ONLINE", C.green, 1)

    drawScope(2, 13, W - 4, H - 15, true)
    txt(5, H - 11, "3 CONTACTS", C.dim, 1)
    txt(math.max(33, W - 25), H - 11, "CLEAR", C.green, 1)
  else
    -- Expanded layout for multi-block displays.
    rect(0, 0, W, 18, C.panel2)
    rect(0, 17, W, 1, C.cyan)
    txt(8, 5, "NODE TELEMETRY", C.text, 1)
    txt(math.max(8, W - 82), 5, "LINK ONLINE", C.green, 1)

    local margin = 8
    local leftW = math.floor(W * 0.68)
    local scopeX, scopeY = margin, 26
    local scopeW, scopeH = leftW - margin * 2, H - 34
    local sideX = leftW + 2
    local sideW = W - sideX - margin

    drawScope(scopeX, scopeY, scopeW, scopeH, false)

    rect(sideX, scopeY, sideW, scopeH, C.panel)
    outline(sideX, scopeY, sideW, scopeH, C.border)
    txt(sideX + 8, scopeY + 8, "STATUS", C.dim, 1)
    txt(sideX + 8, scopeY + 24, "CLEAR", C.green, 2)

    local yy = scopeY + 52
    local function stat(label, value, color)
      if yy + 20 >= H - 30 then return end
      txt(sideX + 8, yy, label, C.dim, 1)
      txt(sideX + 8, yy + 11, value, color or C.text, 1)
      yy = yy + 32
    end

    stat("CONTACTS", #contacts, C.text)
    stat("PRIORITY", "B22", C.red)
    stat("RANGE", "2048", C.cyan)
    stat("CHANNEL", "PRIMARY", C.text)

    if sideW > 24 and H > 70 then
      local barY = H - 18
      txt(sideX + 8, barY - 11, "LOAD", C.dim, 1)
      local bw = math.max(1, sideW - 16)
      rect(sideX + 8, barY, bw, 5, C.grid)
      rect(sideX + 8, barY, math.floor(bw * .37), 5, C.cyan)
    end
  end

  gpu.sync()
  sweep = (sweep + 0.055) % (math.pi * 2)
  sleep(0.05)
end
