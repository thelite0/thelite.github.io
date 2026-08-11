term.clear()
term.setCursorPos(1,1)

assert(sublevel and sublevel.getLogicalPose, "CC:Sable sublevel API unavailable")

local pose = sublevel.getLogicalPose()
print("POSE KEYS:")
for k,v in pairs(pose) do
  print(tostring(k) .. " : " .. type(v))
end

print("")
print("ORIENTATION:")
local q = pose.orientation
print("type=" .. type(q))
if type(q) == "table" then
  for k,v in pairs(q) do
    print("[" .. tostring(k) .. "] = " .. tostring(v) .. " (" .. type(v) .. ")")
  end
else
  print(tostring(q))
end

print("")
print("ANGULAR VELOCITY:")
local a = sublevel.getAngularVelocity()
for k,v in pairs(a) do print("["..tostring(k).."]="..tostring(v)) end
