local argv = {...}
local unpackArgs = table.unpack or unpack

-- Relocation shim. Keeps the previous display/access-control stack pinned,
-- but rewrites its local origin constants before it is loaded.
local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1833, 89, 2458

-- Preserve the same monitored world direction as before.
-- Previous origin 1903/2442 with sector 1881/1632 pointed at world X=22 Z=810.
local SECTOR_X, SECTOR_Z = 1811, 1648

local GUARD_URL = "https://raw.githubusercontent.com/thelite0/thelite.github.io/09a24ca87f5229a95a88a010b90d77c060d1a207/ops/display.lua"
local GUARD_CACHE = ".display_guard_09a.lua"
local BASE_URL = "https://raw.githubusercontent.com/thelite0/thelite.github.io/b7654827709b49d5867b65cc2a3b52720f2853b9/ops/display.lua"
local BASE_CACHE = ".display_base_b765.lua"
local CORE_URL = "https://raw.githubusercontent.com/thelite0/thelite.github.io/ace5f5125d975b4c0c4229c059ac32266b20c7c8/ops/display.lua"
local CORE_CACHE = ".display_core_ace5.lua"

local function readFile(path)
  if not fs.exists(path) then return nil end
  local f = fs.open(path, "r")
  if not f then return nil end
  local body = f.readAll()
  f.close()
  return body
end

local function writeFile(path, body)
  local tmp = path .. ".tmp"
  local f = assert(fs.open(tmp, "w"))
  f.write(body)
  f.close()
  if fs.exists(path) then fs.delete(path) end
  fs.move(tmp, path)
end

local function download(url)
  assert(http and http.get, "HTTP API unavailable")
  local r = assert(http.get(url), "failed to download runtime")
  local body = r.readAll()
  r.close()
  return body
end

local function replacePlain(body, old, new)
  local out, pos, changed = {}, 1, false
  while true do
    local a, b = body:find(old, pos, true)
    if not a then
      out[#out + 1] = body:sub(pos)
      break
    end
    out[#out + 1] = body:sub(pos, a - 1)
    out[#out + 1] = new
    pos = b + 1
    changed = true
  end
  return table.concat(out), changed
end

local function applyPatches(body, patches)
  for _, p in ipairs(patches) do
    local old, new = p[1], p[2]
    if body:find(old, 1, true) then
      body = replacePlain(body, old, new)
    elseif not body:find(new, 1, true) then
      return nil
    end
  end
  return body
end

local function ensurePatched(url, path, patches)
  local body = readFile(path)
  if body then body = applyPatches(body, patches) end
  if not body then
    body = assert(applyPatches(download(url), patches), "runtime patch anchor missing: " .. path)
  end
  writeFile(path, body)
end

local newOrigin = string.format("local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = %d, %d, %d", ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
local newDistance = string.format("local dx,dy,dz = %d-x,%d-y,%d-z", ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
local newSector = string.format("local SECTOR_X, SECTOR_Z = %d, %d", SECTOR_X, SECTOR_Z)

-- Emergency outsider layer: distance shown in DANGER banner.
ensurePatched(GUARD_URL, GUARD_CACHE, {
  {"local dx,dy,dz = 1903-x,97-y,2442-z", newDistance},
})

-- Archive/history layer: world-to-local player distances.
ensurePatched(BASE_URL, BASE_CACHE, {
  {"local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1903, 97, 2442", newOrigin},
})

-- Main tracker: player local coordinates + monitored direction.
ensurePatched(CORE_URL, CORE_CACHE, {
  {"local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1903, 97, 2442", newOrigin},
  {"local SECTOR_X, SECTOR_Z = 1881, 1632", newSector},
})

local guard, err = loadfile(GUARD_CACHE)
if not guard then error(err, 0) end
return guard(unpackArgs(argv))
