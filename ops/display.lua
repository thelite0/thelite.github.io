local argv = {...}
local unpackArgs = table.unpack or unpack

-- Proximity access-control layer. The previous display stays pinned underneath.
local BASE_URL = "https://raw.githubusercontent.com/thelite0/thelite.github.io/b7654827709b49d5867b65cc2a3b52720f2853b9/ops/display.lua"
local BASE_CACHE = ".display_base_b765.lua"
local ACCESS_FILE = ".ops_access.json"
local SIREN_SIDE = "back"

local DEFAULT_ALLOW = {
  "raz3ware",
  "fant_om22",
  "Nanapos",
  "QXm",
  "Skevor",
  "Xero",
}

local nativeWrap = peripheral.wrap
local nativePullEvent = os.pullEvent
local nativeSetOutput = redstone.setOutput

local function lower(s) return string.lower(tostring(s or "")) end
local function now() return os.epoch("utc") / 1000 end

-- Persistent allowlist -------------------------------------------------------
local allowed = {}
local function resetAllow()
  allowed = {}
  for _, name in ipairs(DEFAULT_ALLOW) do allowed[lower(name)] = name end
end
resetAllow()

local function saveAllow()
  local names = {}
  for _, display in pairs(allowed) do names[#names+1] = display end
  table.sort(names, function(a,b) return lower(a) < lower(b) end)
  local data = { version=1, names=names }
  local body = textutils.serializeJSON and textutils.serializeJSON(data) or textutils.serialize(data)
  local tmp = ACCESS_FILE .. ".tmp"
  local f = assert(fs.open(tmp, "w")); f.write(body); f.close()
  if fs.exists(ACCESS_FILE) then fs.delete(ACCESS_FILE) end
  fs.move(tmp, ACCESS_FILE)
end

local function loadAllow()
  if not fs.exists(ACCESS_FILE) then saveAllow(); return end
  local ok, data = pcall(function()
    local f = fs.open(ACCESS_FILE, "r"); if not f then return nil end
    local body = f.readAll(); f.close()
    return textutils.unserializeJSON and textutils.unserializeJSON(body) or textutils.unserialize(body)
  end)
  if not ok or type(data) ~= "table" or type(data.names) ~= "table" then return end
  allowed = {}
  for _, name in ipairs(data.names) do
    if type(name) == "string" and name ~= "" then allowed[lower(name)] = name end
  end
end
loadAllow()

local function isAllowed(name)
  return allowed[lower(name)] ~= nil
end

-- Locate the bitmap controller before installing wrappers. ------------------
local gpuName, actualGpu
for _, name in ipairs(peripheral.getNames()) do
  local p = nativeWrap(name)
  if p and type(p.refreshSize) == "function" and type(p.sync) == "function" and type(p.getSize) == "function" then
    gpuName, actualGpu = name, p
    break
  end
end
assert(actualGpu, "display controller not found")

local COL = {
  panel=0xFF0B1216, panel2=0xFF101B20, border=0xFF31515A,
  text=0xFFE7F0F2, dim=0xFF758990, green=0xFF4FE39B,
  red=0xFFFF5C68, button=0xFF16242A, buttonOn=0xFF234552,
  white=0xFFFFFFFF,
}

local function rect(x,y,w,h,c) actualGpu.filledRectangle(math.floor(x),math.floor(y),math.floor(w),math.floor(h),c) end
local function outline(x,y,w,h,c) actualGpu.rectangle(math.floor(x),math.floor(y),math.floor(w),math.floor(h),c) end
local function text(x,y,s,c,size)
  pcall(actualGpu.drawText, math.floor(x), math.floor(y), tostring(s or ""), c or COL.text, 0x00000000, size or 1)
end
local function center(x,y,w,s,c,size)
  size=size or 1
  local width=#tostring(s or "") * 6 * size
  text(x + math.floor((w-width)/2), y, s, c, size)
end

-- Intruder state -------------------------------------------------------------
local lastPlayers = {}
local intruders = {}
local intruderWanted = false
local coreWanted = false

local function playerName(v, i)
  local n = v and v.username
  if type(n) == "string" and n ~= "" then return n end
  return "UNKNOWN-" .. tostring(i)
end

local function playerDistance(v)
  local x,y,z = tonumber(v and v.x), tonumber(v and v.y), tonumber(v and v.z)
  if not x or not y or not z then return nil end
  local dx,dy,dz = 1903-x,97-y,2442-z
  return math.sqrt(dx*dx + dy*dy + dz*dz)
end

local function applySiren()
  nativeSetOutput(SIREN_SIDE, coreWanted or intruderWanted)
end

local function rebuildThreat(raw)
  lastPlayers = type(raw)=="table" and raw or {}
  local nextIntruders = {}
  for i,v in ipairs(lastPlayers) do
    local name = playerName(v,i)
    if not isAllowed(name) then
      nextIntruders[lower(name)] = {
        name=name,
        distance=playerDistance(v),
        seenAt=now(),
      }
    end
  end
  intruders = nextIntruders
  intruderWanted = next(intruders) ~= nil
  applySiren()
end

local function primaryIntruder()
  local best, count = nil, 0
  for _,v in pairs(intruders) do
    count = count + 1
    if not best or (v.distance or 1e30) < (best.distance or 1e30) then best = v end
  end
  return best, count
end

-- Siren OR gate: core alerts still work, but a non-allowed person cannot be
-- silenced by the core's normal mute/ack logic.
local function guardedSetOutput(side, value)
  if side == SIREN_SIDE then
    coreWanted = value and true or false
    return nativeSetOutput(side, coreWanted or intruderWanted)
  end
  return nativeSetOutput(side, value)
end
redstone.setOutput = guardedSetOutput
if type(rs) == "table" then rs.setOutput = guardedSetOutput end

-- Touchscreen allowlist UI ---------------------------------------------------
local wlOpen = false
local wlOffset = 1
local wlHits = {}
local currentNames = {}

local function readArchiveNames(out)
  if not fs.exists(".ops_player_records.json") then return end
  pcall(function()
    local f=fs.open(".ops_player_records.json","r"); if not f then return end
    local body=f.readAll(); f.close()
    local data=textutils.unserializeJSON and textutils.unserializeJSON(body) or textutils.unserialize(body)
    if type(data)=="table" and type(data.players)=="table" then
      for key,r in pairs(data.players) do
        local name=(type(r)=="table" and r.name) or key
        if type(name)=="string" and name~="" then out[lower(name)]=name end
      end
    end
  end)
end

local function whitelistRows()
  local map = {}
  for k,v in pairs(allowed) do map[k]=v end
  for k,v in pairs(currentNames) do map[k]=v end
  readArchiveNames(map)
  local rows = {}
  for _,name in pairs(map) do rows[#rows+1]=name end
  table.sort(rows,function(a,b)
    local aa,bb=isAllowed(a),isAllowed(b)
    if aa~=bb then return aa end
    return lower(a)<lower(b)
  end)
  return rows
end

local function addHit(id,x,y,w,h,data)
  wlHits[#wlHits+1]={id=id,x=x,y=y,w=w,h=h,data=data}
end
local function hit(h,x,y)
  return x>=h.x and x<h.x+h.w and y>=h.y and y<h.y+h.h
end
local function smallButton(id,x,y,w,h,label,active,data)
  rect(x,y,w,h,active and COL.buttonOn or COL.button)
  outline(x,y,w,h,active and COL.green or COL.border)
  center(x,y+3,w,label,active and COL.green or COL.text,1)
  addHit(id,x,y,w,h,data)
end

local function drawTopButton()
  local W=select(1,actualGpu.getSize()); if not W or W<=0 then return end
  local x=W-68
  rect(x,8,28,12,wlOpen and COL.buttonOn or COL.button)
  outline(x,8,28,12,wlOpen and COL.green or COL.border)
  center(x,11,28,"WL",wlOpen and COL.green or COL.text,1)
end

local function drawDanger()
  if not intruderWanted then return end
  local W=select(1,actualGpu.getSize()); if not W or W<=0 then return end
  local p,count=primaryIntruder(); if not p then return end
  rect(4,23,W-8,17,0xFF681B23)
  outline(4,23,W-8,17,COL.red)
  local suffix = count>1 and (" +"..tostring(count-1)) or ""
  local d = p.distance and string.format(" %.0fm",p.distance) or ""
  local msg = "DANGER " .. tostring(p.name) .. d .. suffix
  if #msg>28 then msg=msg:sub(1,28) end
  center(4,28,W-8,msg,COL.white,1)
end

local function renderWhitelist()
  local W,H=actualGpu.getSize(); if not W or W<=0 then return end
  wlHits={}
  local x,y,w,h=6,46,W-12,H-82
  rect(x,y,w,h,COL.panel); outline(x,y,w,h,COL.border)
  text(x+6,y+5,"WHITELIST",COL.text,1)
  smallButton("close",x+w-36,y+3,31,12,"BACK",false)
  smallButton("reset",x+w-76,y+3,36,12,"RESET",false)

  local rows=whitelistRows()
  local rowCount=7
  local maxOffset=math.max(1,#rows-rowCount+1)
  wlOffset=math.max(1,math.min(wlOffset,maxOffset))
  local yy=y+23
  for i=wlOffset,math.min(#rows,wlOffset+rowCount-1) do
    local name=rows[i]
    local ok=isAllowed(name)
    local c=ok and COL.green or COL.red
    text(x+6,yy,name,c,1)
    text(x+w-45,yy,ok and "ALLOW" or "ALARM",c,1)
    addHit("toggle",x+4,yy-2,w-8,10,name)
    yy=yy+11
  end
  if #rows==0 then text(x+6,y+35,"NO RECORDS",COL.dim,1) end
  smallButton("prev",x+6,y+h-15,34,11,"PREV",false)
  smallButton("next",x+43,y+h-15,34,11,"NEXT",false)
  text(x+84,y+h-12,tostring(#rows).." KNOWN",COL.dim,1)
  drawTopButton()
  drawDanger()
end

local function topButtonHit(x,y)
  local W=select(1,actualGpu.getSize())
  return x>=W-68 and x<W-40 and y>=8 and y<20
end

local function reevaluate()
  rebuildThreat(lastPlayers)
end

local function handleWhitelistTouch(x,y)
  for i=#wlHits,1,-1 do
    local h=wlHits[i]
    if hit(h,x,y) then
      if h.id=="close" then
        wlOpen=false
      elseif h.id=="reset" then
        resetAllow(); saveAllow(); reevaluate(); wlOffset=1
      elseif h.id=="toggle" then
        local key=lower(h.data)
        if allowed[key] then allowed[key]=nil else allowed[key]=h.data end
        saveAllow(); reevaluate()
      elseif h.id=="prev" then
        wlOffset=math.max(1,wlOffset-7)
      elseif h.id=="next" then
        wlOffset=wlOffset+7
      end
      return true
    end
  end
  return false
end

-- GPU proxy sits underneath the existing UI. Its sync runs last, so emergency
-- status and the allowlist panel remain visible even while the core redraws.
local guardGpu={}
setmetatable(guardGpu,{__index=function(_,k)
  local v=actualGpu[k]
  if type(v)=="function" then return function(...) return v(...) end end
  return v
end})
guardGpu.sync=function(...)
  drawTopButton()
  drawDanger()
  if wlOpen then renderWhitelist() end
  return actualGpu.sync(...)
end

local peripheralCache={}
peripheral.wrap=function(name)
  if name==gpuName then return guardGpu end
  if peripheralCache[name] then return peripheralCache[name] end
  local raw=nativeWrap(name); if not raw then return nil end
  if type(raw.scanForPlayers)=="function" then
    local p={}
    setmetatable(p,{__index=function(_,k)
      local v=raw[k]
      if type(v)=="function" then return function(...) return v(...) end end
      return v
    end})
    p.scanForPlayers=function(radius)
      local result=raw.scanForPlayers(radius)
      currentNames={}
      if type(result)=="table" then
        for i,v in ipairs(result) do
          local n=playerName(v,i); currentNames[lower(n)]=n
        end
        rebuildThreat(result)
      else
        rebuildThreat({})
      end
      return result
    end
    peripheralCache[name]=p
    return p
  end
  return raw
end

-- This becomes the previous layer's native event source. WL touches are
-- consumed here; timers and all other events continue to the main display.
os.pullEvent=function(filter)
  while true do
    local ev={nativePullEvent(filter)}
    if ev[1]=="tm_monitor_touch" then
      local x,y=ev[3],ev[4]
      if topButtonHit(x,y) then
        wlOpen=not wlOpen; wlOffset=1
        if wlOpen then renderWhitelist() else drawTopButton() end
        actualGpu.sync()
      elseif wlOpen then
        handleWhitelistTouch(x,y)
        if wlOpen then renderWhitelist() end
        actualGpu.sync()
      else
        return unpackArgs(ev)
      end
    else
      return unpackArgs(ev)
    end
  end
end

local function ensureBase()
  if fs.exists(BASE_CACHE) then return end
  assert(http and http.get,"HTTP API unavailable")
  local r=assert(http.get(BASE_URL),"failed to download display base")
  local body=r.readAll(); r.close()
  local f=assert(fs.open(BASE_CACHE,"w")); f.write(body); f.close()
end

ensureBase()
local base,err=loadfile(BASE_CACHE)
if not base then error(err,0) end
local ok,res=pcall(base,unpackArgs(argv))
-- Do not leave a stale emergency output if the program exits normally.
intruderWanted=false
applySiren()
if not ok then error(res,0) end
return res
