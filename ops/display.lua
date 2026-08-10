local argv = {...}

-- The controller's native text renderer only contains a small Latin glyph set.
-- This shim adds a compact 5x7 Cyrillic raster font while preserving native
-- rendering for ASCII characters. The application core remains version-pinned.

local CORE_URL = "https://raw.githubusercontent.com/thelite0/thelite.github.io/ace5f5125d975b4c0c4229c059ac32266b20c7c8/ops/display.lua"
local CORE_CACHE = ".display_core_ace5.lua"

local FONT = {
  ["А"]={"01110","10001","10001","11111","10001","10001","10001"},
  ["Б"]={"11111","10000","10000","11110","10001","10001","11110"},
  ["В"]={"11110","10001","10001","11110","10001","10001","11110"},
  ["Г"]={"11111","10000","10000","10000","10000","10000","10000"},
  ["Д"]={"00110","01010","01010","01010","01010","11111","10001"},
  ["Е"]={"11111","10000","10000","11110","10000","10000","11111"},
  ["Ё"]={"01010","11111","10000","11110","10000","10000","11111"},
  ["Ж"]={"10101","10101","01110","00100","01110","10101","10101"},
  ["З"]={"11110","00001","00001","01110","00001","00001","11110"},
  ["И"]={"10001","10011","10101","10101","11001","10001","10001"},
  ["Й"]={"01010","00100","10001","10011","10101","11001","10001"},
  ["К"]={"10001","10010","10100","11000","10100","10010","10001"},
  ["Л"]={"00111","01001","01001","01001","01001","10001","10001"},
  ["М"]={"10001","11011","10101","10101","10001","10001","10001"},
  ["Н"]={"10001","10001","10001","11111","10001","10001","10001"},
  ["О"]={"01110","10001","10001","10001","10001","10001","01110"},
  ["П"]={"11111","10001","10001","10001","10001","10001","10001"},
  ["Р"]={"11110","10001","10001","11110","10000","10000","10000"},
  ["С"]={"01111","10000","10000","10000","10000","10000","01111"},
  ["Т"]={"11111","00100","00100","00100","00100","00100","00100"},
  ["У"]={"10001","10001","01010","00100","00100","01000","10000"},
  ["Ф"]={"00100","01110","10101","10101","10101","01110","00100"},
  ["Х"]={"10001","01010","00100","00100","00100","01010","10001"},
  ["Ц"]={"10010","10010","10010","10010","10010","11111","00001"},
  ["Ч"]={"10001","10001","10001","01111","00001","00001","00001"},
  ["Ш"]={"10101","10101","10101","10101","10101","10101","11111"},
  ["Щ"]={"10101","10101","10101","10101","10101","11111","00001"},
  ["Ъ"]={"11000","01000","01110","01001","01001","01001","01110"},
  ["Ы"]={"10001","10001","10101","11001","10101","10001","10001"},
  ["Ь"]={"10000","10000","11110","10001","10001","10001","11110"},
  ["Э"]={"11110","00001","00001","01111","00001","00001","11110"},
  ["Ю"]={"10110","11001","11001","11101","11001","11001","10110"},
  ["Я"]={"01111","10001","10001","01111","00101","01001","10001"},
}

local FALLBACK = {"01110","10001","00010","00100","00100","00000","00100"}

local function eachChar(s)
  s = tostring(s or "")
  local i = 1
  return function()
    if i > #s then return nil end
    local b = s:byte(i)
    local n
    if b < 0x80 then n = 1
    elseif b < 0xE0 then n = 2
    elseif b < 0xF0 then n = 3
    else n = 4 end
    local ch = s:sub(i, i + n - 1)
    i = i + n
    return ch, b
  end
end

local function drawBitmap(raw, x, y, glyph, color, scale)
  scale = math.max(1, math.floor(tonumber(scale) or 1))
  for row = 1, 7 do
    local bits = glyph[row]
    for col = 1, 5 do
      if bits:sub(col, col) == "1" then
        raw.filledRectangle(
          math.floor(x + (col - 1) * scale),
          math.floor(y + (row - 1) * scale),
          scale,
          scale,
          color
        )
      end
    end
  end
end

local function rasterText(raw, x, y, s, color, background, scale)
  x = math.floor(tonumber(x) or 0)
  y = math.floor(tonumber(y) or 0)
  scale = math.max(1, math.floor(tonumber(scale) or 1))
  local cursor = x

  for ch, firstByte in eachChar(s) do
    if firstByte < 0x80 then
      raw.drawText(cursor, y, ch, color, background or 0x00000000, scale)
    else
      drawBitmap(raw, cursor, y, FONT[ch] or FALLBACK, color, scale)
    end
    cursor = cursor + 6 * scale
  end
end

local realWrap = peripheral.wrap
local proxies = {}

peripheral.wrap = function(name)
  local cached = proxies[name]
  if cached then return cached end

  local raw = realWrap(name)
  if not raw then return nil end

  if type(raw.getSize) == "function"
      and type(raw.sync) == "function"
      and type(raw.drawText) == "function"
      and type(raw.filledRectangle) == "function" then
    local proxy = setmetatable({}, {__index = raw})
    proxy.drawText = function(x, y, text, color, background, scale)
      return rasterText(raw, x, y, text, color, background, scale)
    end
    proxies[name] = proxy
    return proxy
  end

  return raw
end

local function ensureCore()
  if fs.exists(CORE_CACHE) then return end
  assert(http and http.get, "HTTP API unavailable; cannot load display core")
  local response, err = http.get(CORE_URL)
  assert(response, "failed to load display core: " .. tostring(err))
  local body = response.readAll()
  response.close()
  local f = fs.open(CORE_CACHE, "w")
  assert(f, "cannot create display core cache")
  f.write(body)
  f.close()
end

ensureCore()
local core, err = loadfile(CORE_CACHE)
assert(core, err)
local unpackArgs = table.unpack or unpack
return core(unpackArgs(argv))
