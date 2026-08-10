local argv = {...}
local unpackArgs = table.unpack or unpack

-- Relocation + noise-hardening shim. Keeps the previous display/access-control
-- stack pinned, but rewrites local origin constants and hardens motion tracking.
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

local function writeFreshPatched(url, path, patches)
  local body = download(url)
  body = assert(applyPatches(body, patches), "runtime patch anchor missing: " .. path)
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

-- Survival radar has large randomized spread. The old filter used the expected
-- noise only, which is not enough when the actual Sable pose itself jitters.
-- This version measures the observed residual spread, requires a statistically
-- significant trend, requires the first/last windows to agree, then requires
-- several consecutive scans before exposing a non-zero closing speed.
local filterAnchor = "local function totalSpeed(h)"
local filterCode = [[
local RADAR_NOISE_FRACTION = 1 / (math.pi * math.pi * math.pi)
local SUB_NOISE_MIN_SAMPLES = 12
local SUB_NOISE_MIN_WINDOW = 7.5
local SUB_NOISE_MIN_Z = 4.5
local SUB_CONFIRM_SCANS = 3

local function meanRange(h, a, b, field)
  local sum, n = 0, 0
  for i=a,b do
    local v = h[i] and h[i][field]
    if type(v) == "number" then sum = sum + v; n = n + 1 end
  end
  return n > 0 and (sum / n) or 0
end

local function filteredSubClosing(h)
  local n = #h
  if n < SUB_NOISE_MIN_SAMPLES then return 0 end

  local window = h[n].t - h[1].t
  if window < SUB_NOISE_MIN_WINDOW then return 0 end

  local mt, md = 0, 0
  for _, p in ipairs(h) do
    mt = mt + p.t
    md = md + math.abs(p.d or 0)
  end
  mt, md = mt / n, md / n

  local sxx, sxy = 0, 0
  for _, p in ipairs(h) do
    local dt = p.t - mt
    sxx = sxx + dt * dt
    sxy = sxy + dt * ((p.d or 0) - md)
  end
  if sxx <= 0.0001 then return 0 end

  local slope = sxy / sxx
  local closing = -slope
  if closing <= 0 then return 0 end

  local intercept = md - slope * mt
  local sse = 0
  for _, p in ipairs(h) do
    local predicted = intercept + slope * p.t
    local residual = (p.d or 0) - predicted
    sse = sse + residual * residual
  end

  local observedSigma = math.sqrt(math.max(0, sse / math.max(1, n - 2)))
  local theoreticalSigma = math.max(1, md * RADAR_NOISE_FRACTION / math.sqrt(3))
  local sigma = math.max(1, observedSigma, theoreticalSigma)
  local slopeSigma = sigma / math.sqrt(sxx)
  if slopeSigma <= 0 then return 0 end
  if closing / slopeSigma < SUB_NOISE_MIN_Z then return 0 end

  -- Independent sanity check: the average of the oldest third must be clearly
  -- farther away than the newest third. This kills lucky regression slopes.
  local k = math.max(3, math.floor(n / 3))
  local oldMean = meanRange(h, 1, k, "d")
  local newMean = meanRange(h, n-k+1, n, "d")
  local delta = oldMean - newMean
  local deltaSigma = sigma * math.sqrt(2 / k)
  if delta <= math.max(12, 3.5 * deltaSigma) then return 0 end

  return closing
end

local function smoothSubSample(h)
  local n = #h
  if n == 0 then return {x=0,y=0,z=0,d=0} end
  local first = math.max(1, n - 4)
  local count = n - first + 1
  local x,y,z,d = 0,0,0,0
  for i=first,n do
    local p=h[i]
    x=x+(p.x or 0); y=y+(p.y or 0); z=z+(p.z or 0); d=d+(p.d or 0)
  end
  return {x=x/count,y=y/count,z=z/count,d=d/count}
end

local function smoothSubHistory(h)
  local out = {}
  for i=1,#h do
    local a,b=math.max(1,i-2),math.min(#h,i+2)
    local x,y,z,d=0,0,0,0
    local count=b-a+1
    for j=a,b do
      local p=h[j]
      x=x+(p.x or 0); y=y+(p.y or 0); z=z+(p.z or 0); d=d+(p.d or 0)
    end
    out[#out+1]={t=h[i].t,x=x/count,y=y/count,z=z/count,d=d/count}
  end
  return out
end

local function totalSpeed(h)]]

local oldSubBlock = [[local tr = ensureTrack(subTracks, key)
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
    }]]

local newSubBlock = [[local tr = ensureTrack(subTracks, key)
    addSample(tr, sample, now)
    local stable = smoothSubSample(tr.history)
    local candidateClosing = filteredSubClosing(tr.history)
    local threshold = stable.d > 1500 and 10 or stable.d > 900 and 6 or 3
    if candidateClosing > threshold then
      tr.inboundConfirm = math.min(SUB_CONFIRM_SCANS, (tr.inboundConfirm or 0) + 1)
    else
      tr.inboundConfirm = math.max(0, (tr.inboundConfirm or 0) - 1)
    end
    local closing = tr.inboundConfirm >= SUB_CONFIRM_SCANS and candidateClosing or 0
    local row = {
      kind = "sub",
      key = key,
      name = v.name or "АППАРАТ",
      x = stable.x, y = stable.y, z = stable.z, d = stable.d,
      closing = closing,
      speed = closing > 0 and totalSpeed(smoothSubHistory(tr.history)) or 0,
      eta = closing > 1 and stable.d / closing or nil,
      sector = inSector(stable.x, stable.z),
      history = smoothSubHistory(tr.history),
    }]]

-- Main tracker is regenerated from the pinned clean core every start. This also
-- avoids repeatedly injecting the same patch into an already-patched cache.
writeFreshPatched(CORE_URL, CORE_CACHE, {
  {"local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1903, 97, 2442", newOrigin},
  {"local SECTOR_X, SECTOR_Z = 1881, 1632", newSector},
  {"local HISTORY = 10", "local HISTORY = 16"},
  {"local SUB_MIN_SAMPLES = 4", "local SUB_MIN_SAMPLES = 12"},
  {filterAnchor, filterCode},
  {oldSubBlock, newSubBlock},
})

local guard, err = loadfile(GUARD_CACHE)
if not guard then error(err, 0) end
return guard(unpackArgs(argv))
