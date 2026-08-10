local argv = {...}
local unpackArgs = table.unpack or unpack

-- Interactive Russian UI shim + persistent player archive.
-- The main application stays pinned; this file adds Cyrillic rendering and
-- archives every player returned by the personnel sensor.
local CORE_URL = "https://raw.githubusercontent.com/thelite0/thelite.github.io/ace5f5125d975b4c0c4229c059ac32266b20c7c8/ops/display.lua"
local CORE_CACHE = ".display_core_ace5.lua"
local RECORD_FILE = ".ops_player_records.json"
local RECORD_SAVE_INTERVAL = 5
local PERSON_RANGE = 1024
local ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1903, 97, 2442

local nativeWrap = peripheral.wrap
local nativePullEvent = os.pullEvent

local gpuName, rawGpu
for _, name in ipairs(peripheral.getNames()) do
  local p = nativeWrap(name)
  if p and type(p.refreshSize) == "function" and type(p.sync) == "function" and type(p.getSize) == "function" then
    gpuName, rawGpu = name, p
    break
  end
end
assert(rawGpu, "display controller not found")

local C = {
  panel=0xFF0B1216, panel2=0xFF101B20, border=0xFF31515A, text=0xFFE7F0F2,
  dim=0xFF758990, cyan=0xFF49D9F2, blue=0xFF6AA6FF, green=0xFF4FE39B,
  yellow=0xFFF0D15A, red=0xFFFF5C68, button=0xFF16242A, buttonOn=0xFF234552,
}

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
  ["У"]={"10001","10001","10001","01111","00001","00001","11110"},
  ["Ф"]={"00100","11111","10101","10101","11111","00100","00100"},
  ["Х"]={"10001","10001","01010","00100","01010","10001","10001"},
  ["Ц"]={"10010","10010","10010","10010","10010","11111","00001"},
  ["Ч"]={"10001","10001","10001","01111","00001","00001","00001"},
  ["Ш"]={"10101","10101","10101","10101","10101","10101","11111"},
  ["Щ"]={"10101","10101","10101","10101","10101","11111","00001"},
  ["Ъ"]={"11000","01000","01000","01110","01001","01001","01110"},
  ["Ы"]={"10001","10001","10001","11101","10101","10101","11101"},
  ["Ь"]={"10000","10000","10000","11110","10001","10001","11110"},
  ["Э"]={"11110","00001","00001","01111","00001","00001","11110"},
  ["Ю"]={"10110","11001","11001","11101","11001","11001","10110"},
  ["Я"]={"01111","10001","10001","01111","00101","01001","10001"},
}
local LOWER = {
  ["а"]="А",["б"]="Б",["в"]="В",["г"]="Г",["д"]="Д",["е"]="Е",["ё"]="Ё",["ж"]="Ж",
  ["з"]="З",["и"]="И",["й"]="Й",["к"]="К",["л"]="Л",["м"]="М",["н"]="Н",["о"]="О",
  ["п"]="П",["р"]="Р",["с"]="С",["т"]="Т",["у"]="У",["ф"]="Ф",["х"]="Х",["ц"]="Ц",
  ["ч"]="Ч",["ш"]="Ш",["щ"]="Щ",["ъ"]="Ъ",["ы"]="Ы",["ь"]="Ь",["э"]="Э",["ю"]="Ю",["я"]="Я",
}

local function chars(s)
  s=tostring(s or "")
  local out,i={},1
  while i<=#s do
    local b=s:byte(i); local n=1
    if b>=0xF0 then n=4 elseif b>=0xE0 then n=3 elseif b>=0xC0 then n=2 end
    out[#out+1]=s:sub(i,i+n-1); i=i+n
  end
  return out
end
local function ulen(s) return #chars(s) end
local function usub(s,a,b)
  local c=chars(s); a=math.max(1,a or 1); b=math.min(#c,b or #c); local out={}
  for i=a,b do out[#out+1]=c[i] end
  return table.concat(out)
end
local function short(s,n)
  s=tostring(s or "НЕИЗВ.")
  if ulen(s)<=n then return s end
  return usub(s,1,math.max(1,n-3)).."..."
end
local function rrect(x,y,w,h,color) rawGpu.filledRectangle(math.floor(x),math.floor(y),math.floor(w),math.floor(h),color) end
local function routline(x,y,w,h,color) rawGpu.rectangle(math.floor(x),math.floor(y),math.floor(w),math.floor(h),color) end
local function drawGlyph(x,y,ch,color,scale)
  local g=FONT[LOWER[ch] or ch]; if not g then return false end
  scale=scale or 1
  for ry,row in ipairs(g) do
    for rx=1,5 do
      if row:sub(rx,rx)=="1" then rrect(x+(rx-1)*scale,y+(ry-1)*scale,scale,scale,color) end
    end
  end
  return true
end
local function drawText(x,y,s,color,bg,size)
  size=size or 1; color=color or C.text; local cx=math.floor(x)
  for _,ch in ipairs(chars(tostring(s))) do
    if FONT[ch] or LOWER[ch] then drawGlyph(cx,y,ch,color,size)
    else pcall(rawGpu.drawText,cx,y,ch,color,bg or 0x00000000,size) end
    cx=cx+6*size
  end
end
local function textWidth(s,size) return ulen(s)*6*(size or 1) end
local function centered(y,s,color,size,x,w)
  x=x or 0; w=w or select(1,rawGpu.getSize()); drawText(x+math.floor((w-textWidth(s,size))/2),y,s,color,0x00000000,size)
end

-- Persistent player records.
local db={version=1,players={}}
local dirty=false
local lastSave=0
local present={}
local motion={}
local function now() return os.epoch("utc")/1000 end
local function loadDb()
  if not fs.exists(RECORD_FILE) then return end
  local ok,data=pcall(function()
    local f=fs.open(RECORD_FILE,"r"); if not f then return nil end
    local s=f.readAll(); f.close()
    return textutils.unserializeJSON and textutils.unserializeJSON(s) or textutils.unserialize(s)
  end)
  if ok and type(data)=="table" and type(data.players)=="table" then db=data end
end
local function saveDb(force)
  local t=now(); if not dirty then return end
  if not force and t-lastSave<RECORD_SAVE_INTERVAL then return end
  local ok=pcall(function()
    local s=textutils.serializeJSON and textutils.serializeJSON(db) or textutils.serialize(db)
    local tmp=RECORD_FILE..".tmp"; local f=assert(fs.open(tmp,"w")); f.write(s); f.close()
    if fs.exists(RECORD_FILE) then fs.delete(RECORD_FILE) end; fs.move(tmp,RECORD_FILE)
  end)
  if ok then dirty=false; lastSave=t end
end
loadDb()

local function distance(wx,wy,wz)
  local x,y,z=ORIGIN_X-wx,ORIGIN_Y-wy,ORIGIN_Z-wz
  return math.sqrt(x*x+y*y+z*z)
end
local function recordPlayers(raw)
  local t=now(); local current={}
  for i,v in ipairs(raw or {}) do
    local name=tostring(v.username or ("UNKNOWN-"..i)); current[name]=true
    local wx,wy,wz=v.x or ORIGIN_X,v.y or ORIGIN_Y,v.z or ORIGIN_Z; local d=distance(wx,wy,wz)
    local r=db.players[name]; local fresh=false
    if not r then r={name=name,firstSeen=t,lastSeen=t,sessions=1,samples=0,observed=0,closest=d,maxSpeed=0,maxClosing=0}; db.players[name]=r; fresh=true end
    local m=motion[name]; local speed,closing=0,0
    if m then local dt=t-m.t; if dt>0 and dt<4 then local dx,dy,dz=wx-m.x,wy-m.y,wz-m.z; speed=math.sqrt(dx*dx+dy*dy+dz*dz)/dt; closing=(m.d-d)/dt; r.observed=(r.observed or 0)+dt end end
    if not fresh and not present[name] and (not r.lastSeen or t-r.lastSeen>3) then r.sessions=(r.sessions or 0)+1 end
    r.samples=(r.samples or 0)+1; r.lastSeen=t; r.lastX,r.lastY,r.lastZ=wx,wy,wz; r.lastDistance=d
    r.closest=math.min(r.closest or d,d); r.maxSpeed=math.max(r.maxSpeed or 0,speed); r.maxClosing=math.max(r.maxClosing or 0,closing)
    r.lastHealth=v.health; r.maxHealth=v.maxHealth; motion[name]={x=wx,y=wy,z=wz,d=d,t=t}; dirty=true
  end
  present=current; saveDb(false)
end

-- Kyiv time (EET/EEST, EU-style last-Sunday DST rule).
local function weekday(y,m,d)
  local a={0,3,2,5,0,3,5,1,4,6,2,4}; if m<3 then y=y-1 end
  return (y+math.floor(y/4)-math.floor(y/100)+math.floor(y/400)+a[m]+d)%7
end
local function lastSunday(y,m)
  local days=({31,28,31,30,31,30,31,31,30,31,30,31})[m]
  if m==2 and ((y%4==0 and y%100~=0) or y%400==0) then days=29 end
  return days-weekday(y,m,days)
end
local function kyivOffset(epoch)
  local u=os.date("!*t",epoch)
  if u.month>3 and u.month<10 then return 3 end
  if u.month<3 or u.month>10 then return 2 end
  if u.month==3 then local d=lastSunday(u.year,3); return (u.day>d or (u.day==d and u.hour>=1)) and 3 or 2 end
  local d=lastSunday(u.year,10); return (u.day<d or (u.day==d and u.hour<1)) and 3 or 2
end
local function fmtTime(epoch,full)
  if not epoch then return "--" end
  local t=os.date("!*t",epoch+kyivOffset(epoch)*3600)
  if full then return string.format("%02d.%02d.%02d %02d:%02d",t.day,t.month,t.year%100,t.hour,t.min) end
  return string.format("%02d.%02d %02d:%02d",t.day,t.month,t.hour,t.min)
end
local function fmtDuration(sec)
  sec=math.max(0,math.floor(sec or 0)); local h=math.floor(sec/3600); local m=math.floor((sec%3600)/60); local s=sec%60
  return h>0 and string.format("%d:%02d:%02d",h,m,s) or string.format("%02d:%02d",m,s)
end

-- Archive overlay. It replaces only the body, so the live alert header and
-- footer from the core remain visible and operational.
local historyMode=false
local historySort="last"
local historyOffset=1
local historySelected=nil
local historyHits={}
local function addHit(id,x,y,w,h,data) historyHits[#historyHits+1]={id=id,x=x,y=y,w=w,h=h,data=data} end
local function button(id,x,y,w,h,label,active)
  rrect(x,y,w,h,active and C.buttonOn or C.button); routline(x,y,w,h,active and C.cyan or C.border); centered(y+3,label,active and C.cyan or C.text,1,x,w); addHit(id,x,y,w,h)
end
local function archiveRows()
  local rows={}; for _,r in pairs(db.players) do rows[#rows+1]=r end
  table.sort(rows,function(a,b)
    if historySort=="name" then return tostring(a.name):lower()<tostring(b.name):lower() end
    if historySort=="closest" then local x,y=a.closest or 1e9,b.closest or 1e9; if x~=y then return x<y end end
    if historySort=="sessions" then local x,y=a.sessions or 0,b.sessions or 0; if x~=y then return x>y end end
    return (a.lastSeen or 0)>(b.lastSeen or 0)
  end)
  return rows
end
local function archiveButton(active)
  local w=select(1,rawGpu.getSize()); if not w or w<=0 then return end
  local x=w-34; local y=8
  rrect(x,y,28,12,active and C.buttonOn or C.button); routline(x,y,28,12,active and C.cyan or C.border); centered(y+3,"АРХ",active and C.cyan or C.text,1,x,28)
end
local function renderArchive()
  local W,H=rawGpu.getSize(); if not W or W<=0 then return end
  historyHits={}; local x,y,w,h=6,46,W-12,H-82
  rrect(x,y,w,h,C.panel); routline(x,y,w,h,C.border)
  local rows=archiveRows(); drawText(x+6,y+5,"АРХИВ ИГРОКОВ: "..#rows,C.text,0x00000000,1)
  if historySelected then
    local r=db.players[historySelected]
    if not r then historySelected=nil else
      button("back",x+w-31,y+3,26,12,"НАЗ",false)
      local active=present[r.name] and true or false
      drawText(x+6,y+20,short(r.name,18),active and C.green or C.blue,0x00000000,1)
      drawText(x+6,y+32,active and "СЕЙЧАС: В РАДИУСЕ" or "СЕЙЧАС: НЕ ВИДЕН",active and C.green or C.dim,0x00000000,1)
      drawText(x+6,y+44,"ПЕРВЫЙ: "..fmtTime(r.firstSeen,true),C.text,0x00000000,1)
      drawText(x+6,y+56,"ПОСЛЕД.: "..fmtTime(r.lastSeen,true),C.text,0x00000000,1)
      drawText(x+6,y+68,string.format("СЕАНСЫ: %d  ВРЕМЯ: %s",r.sessions or 0,fmtDuration(r.observed or 0)),C.dim,0x00000000,1)
      drawText(x+6,y+80,string.format("МИН.ДИСТ: %.0f  МАКС.V: %.1f",r.closest or 0,r.maxSpeed or 0),C.text,0x00000000,1)
      drawText(x+6,y+92,string.format("МАКС.СБЛ: %.1f  HP: %.0f/%.0f",r.maxClosing or 0,r.lastHealth or 0,r.maxHealth or 0),C.text,0x00000000,1)
      drawText(x+6,y+104,string.format("КООРД: %.0f %.0f %.0f",r.lastX or 0,r.lastY or 0,r.lastZ or 0),C.dim,0x00000000,1)
      archiveButton(true); return
    end
  end
  button("sort_last",x+6,y+16,34,12,"ПОСЛ",historySort=="last")
  button("sort_name",x+43,y+16,34,12,"ИМЯ",historySort=="name")
  button("sort_close",x+80,y+16,34,12,"БЛИЗ",historySort=="closest")
  button("sort_sessions",x+117,y+16,34,12,"СЕАН",historySort=="sessions")
  local rowCount=7; local maxOffset=math.max(1,#rows-rowCount+1); historyOffset=math.max(1,math.min(historyOffset,maxOffset)); local yy=y+34
  for i=historyOffset,math.min(#rows,historyOffset+rowCount-1) do
    local r=rows[i]; local active=present[r.name] and true or false
    drawText(x+6,yy,active and "*" or "-",active and C.green or C.dim,0x00000000,1)
    drawText(x+16,yy,short(r.name,10),active and C.green or C.blue,0x00000000,1)
    drawText(x+82,yy,fmtTime(r.lastSeen,false),C.text,0x00000000,1)
    drawText(x+156,yy,r.lastDistance and string.format("%.0f",r.lastDistance) or "--",C.dim,0x00000000,1)
    addHit("row",x+4,yy-2,w-8,10,r.name); yy=yy+10
  end
  if #rows==0 then centered(y+58,"АРХИВ ПОКА ПУСТ",C.dim,1,x,w) end
  button("prev",x+6,y+h-15,34,11,"ПРЕД",false); button("next",x+43,y+h-15,34,11,"СЛЕД",false)
  drawText(x+84,y+h-12,"КИЕВСКОЕ ВРЕМЯ",C.dim,0x00000000,1); archiveButton(true)
end
local function archiveButtonHit(x,y)
  local W=select(1,rawGpu.getSize()); return x>=W-34 and x<W-6 and y>=8 and y<20
end
local function handleArchiveClick(x,y)
  for i=#historyHits,1,-1 do
    local h=historyHits[i]
    if x>=h.x and x<h.x+h.w and y>=h.y and y<h.y+h.h then
      if h.id=="row" then historySelected=h.data
      elseif h.id=="back" then historySelected=nil
      elseif h.id=="sort_last" then historySort="last"; historyOffset=1
      elseif h.id=="sort_name" then historySort="name"; historyOffset=1
      elseif h.id=="sort_close" then historySort="closest"; historyOffset=1
      elseif h.id=="sort_sessions" then historySort="sessions"; historyOffset=1
      elseif h.id=="prev" then historyOffset=math.max(1,historyOffset-7)
      elseif h.id=="next" then historyOffset=historyOffset+7 end
      renderArchive(); rawGpu.sync(); return true
    end
  end
  return false
end

-- GPU proxy: preserve all methods, replace text renderer, and draw archive overlay.
local gpuProxy={}
setmetatable(gpuProxy,{__index=function(_,k)
  local v=rawGpu[k]
  if type(v)=="function" then return function(...) return v(...) end end
  return v
end})
gpuProxy.drawText=function(x,y,s,color,bg,size) return drawText(x,y,s,color,bg,size) end
gpuProxy.sync=function(...)
  if historyMode then renderArchive() else archiveButton(false) end
  return rawGpu.sync(...)
end

local proxyCache={}
peripheral.wrap=function(name)
  if name==gpuName then return gpuProxy end
  if proxyCache[name] then return proxyCache[name] end
  local raw=nativeWrap(name); if not raw then return nil end
  if type(raw.scanForPlayers)=="function" then
    local p={}
    setmetatable(p,{__index=function(_,k) local v=raw[k]; if type(v)=="function" then return function(...) return v(...) end end; return v end})
    p.scanForPlayers=function(radius)
      local result=raw.scanForPlayers(radius)
      if type(result)=="table" then recordPlayers(result) else present={} end
      return result
    end
    proxyCache[name]=p; return p
  end
  return raw
end

-- Consume archive clicks while leaving the core's timers/scanning alive.
os.pullEvent=function(filter)
  while true do
    local ev={nativePullEvent(filter)}
    if ev[1]=="tm_monitor_touch" then
      local x,y=ev[3],ev[4]
      if archiveButtonHit(x,y) then
        historyMode=not historyMode; historySelected=nil
        if historyMode then renderArchive() else archiveButton(false) end
        rawGpu.sync()
      elseif historyMode then
        local _,H=rawGpu.getSize()
        if y<46 or y>=H-30 then
          historyMode=false
          return unpackArgs(ev)
        else
          handleArchiveClick(x,y)
        end
      else
        return unpackArgs(ev)
      end
    else
      return unpackArgs(ev)
    end
  end
end

local function ensureCore()
  if fs.exists(CORE_CACHE) then return end
  assert(http and http.get,"HTTP API unavailable")
  local r=assert(http.get(CORE_URL),"failed to download display core")
  local body=r.readAll(); r.close(); local f=assert(fs.open(CORE_CACHE,"w")); f.write(body); f.close()
end

ensureCore()
local core,err=loadfile(CORE_CACHE)
if not core then error(err,0) end
local ok,res=pcall(core,unpackArgs(argv))
saveDb(true)
if not ok then error(res,0) end
return res
