local gpu = peripheral.wrap("bottom")
assert(gpu, "display controller not found on bottom")
assert(gpu.refreshSize and gpu.sync, "bottom peripheral is not a compatible display controller")

gpu.refreshSize()
gpu.setSize(64)

local W, H = gpu.getSize()
assert(type(W) == "number" and type(H) == "number", "could not determine display size")

local C = {
  bg      = 0xFF071016,
  panel   = 0xFF0D1921,
  panel2  = 0xFF101F29,
  grid    = 0xFF173541,
  border  = 0xFF245266,
  text    = 0xFFD7EDF5,
  dim     = 0xFF6E8791,
  cyan    = 0xFF42D5F5,
  green   = 0xFF51E59A,
  yellow  = 0xFFF3CC5C,
  orange  = 0xFFF08A4B,
  red     = 0xFFFF5D68,
}

local function rect(x, y, w, h, c)
  gpu.filledRectangle(math.floor(x), math.floor(y), math.floor(w), math.floor(h), c)
end

local function outline(x, y, w, h, c)
  gpu.rectangle(math.floor(x), math.floor(y), math.floor(w), math.floor(h), c)
end

local function txt(x, y, s, c, size)
  gpu.drawText(math.floor(x), math.floor(y), tostring(s), c or C.text, 0x00000000, size or 1)
end

local function line(x1, y1, x2, y2, c)
  gpu.lineS(math.floor(x1), math.floor(y1), math.floor(x2), math.floor(y2), c)
end

local function circle(cx, cy, r, c, segments)
  segments = segments or 48
  local px, py = cx + r, cy
  for i = 1, segments do
    local a = (i / segments) * math.pi * 2
    local x, y = cx + math.cos(a) * r, cy + math.sin(a) * r
    line(px, py, x, y, c)
    px, py = x, y
  end
end

local function clamp(v, a, b)
  if v < a then return a end
  if v > b then return b end
  return v
end

local contacts = {
  {x=.62, y=.24, level=2, id="A-17"},
  {x=-.30, y=-.55, level=1, id="C-04"},
  {x=.18, y=-.15, level=3, id="B-22"},
}

local levelColor = {C.green, C.yellow, C.red}
local sweep = 0

while true do
  gpu.fill(C.bg)

  -- Header
  rect(0, 0, W, 18, C.panel2)
  rect(0, 17, W, 1, C.cyan)
  txt(8, 5, "NODE TELEMETRY", C.text, 1)
  txt(W - 82, 5, "LINK  ONLINE", C.green, 1)

  -- Layout
  local margin = 8
  local leftW = math.floor(W * 0.68)
  local scopeX, scopeY = margin, 26
  local scopeW, scopeH = leftW - margin * 2, H - 34
  local sideX = leftW + 2
  local sideW = W - sideX - margin

  rect(scopeX, scopeY, scopeW, scopeH, C.panel)
  outline(scopeX, scopeY, scopeW, scopeH, C.border)

  rect(sideX, scopeY, sideW, scopeH, C.panel)
  outline(sideX, scopeY, sideW, scopeH, C.border)

  -- Scope
  local cx = scopeX + math.floor(scopeW / 2)
  local cy = scopeY + math.floor(scopeH / 2)
  local r = math.floor(math.min(scopeW, scopeH) * 0.42)

  circle(cx, cy, r, C.border)
  circle(cx, cy, math.floor(r * .66), C.grid)
  circle(cx, cy, math.floor(r * .33), C.grid)
  line(cx-r, cy, cx+r, cy, C.grid)
  line(cx, cy-r, cx, cy+r, C.grid)

  -- Sweep
  local sx = cx + math.cos(sweep) * r
  local sy = cy + math.sin(sweep) * r
  line(cx, cy, sx, sy, C.cyan)
  for d = 1, 4 do
    local a = sweep - d * 0.035
    line(cx, cy, cx + math.cos(a) * r, cy + math.sin(a) * r, 0xFF123847)
  end

  -- Center marker
  rect(cx-2, cy-2, 5, 5, C.cyan)

  -- Contacts
  for _, c in ipairs(contacts) do
    local px = cx + c.x * r
    local py = cy + c.y * r
    local col = levelColor[clamp(c.level, 1, 3)]
    rect(px-3, py-3, 7, 7, col)
    outline(px-5, py-5, 11, 11, col)
    txt(px+8, py-4, c.id, col, 1)
  end

  txt(scopeX + 7, scopeY + 6, "TRACKING FIELD", C.dim, 1)
  txt(scopeX + 7, scopeY + scopeH - 13, string.format("FRAME %04d", math.floor((sweep * 100) % 10000)), C.dim, 1)

  -- Side panel
  txt(sideX + 8, scopeY + 8, "STATUS", C.dim, 1)
  txt(sideX + 8, scopeY + 24, "CLEAR", C.green, 2)

  local yy = scopeY + 52
  local function stat(label, value, color)
    txt(sideX + 8, yy, label, C.dim, 1)
    txt(sideX + 8, yy + 11, value, color or C.text, 1)
    yy = yy + 32
  end

  stat("CONTACTS", #contacts, C.text)
  stat("PRIORITY", "B-22", C.red)
  stat("RANGE", "2048", C.cyan)
  stat("CHANNEL", "PRIMARY", C.text)

  local barY = H - 28
  txt(sideX + 8, barY - 12, "SYSTEM LOAD", C.dim, 1)
  rect(sideX + 8, barY, sideW - 16, 6, C.grid)
  rect(sideX + 8, barY, math.floor((sideW - 16) * .37), 6, C.cyan)

  gpu.sync()
  sweep = (sweep + 0.055) % (math.pi * 2)
  sleep(0.05)
end
