term.clear()
term.setCursorPos(1,1)

local function dump(name, value)
    print("=== " .. name .. " ===")
    print(textutils.serialize(value, { compact = false, allow_repetitions = true }))
    print("")
end

print("CC:Sable probe")
print("")

local okPose, pose = pcall(function() return sublevel.getLogicalPose() end)
if okPose then
    dump("getLogicalPose()", pose)
    if type(pose) == "table" then
        dump("pose.orientation", pose.orientation)
    end
else
    print("getLogicalPose ERROR: " .. tostring(pose))
end

local okAng, ang = pcall(function() return sublevel.getAngularVelocity() end)
if okAng then
    dump("getAngularVelocity()", ang)
else
    print("getAngularVelocity ERROR: " .. tostring(ang))
end

local okLin, lin = pcall(function() return sublevel.getLinearVelocity() end)
if okLin then
    dump("getLinearVelocity()", lin)
else
    print("getLinearVelocity ERROR: " .. tostring(lin))
end
