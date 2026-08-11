local left = peripheral.wrap("tilt_adapter_1")
local right = peripheral.wrap("tilt_adapter_2")

assert(left, "Missing tilt_adapter_1 (front left)")
assert(right, "Missing tilt_adapter_2 (front right)")

-- The two adapters are physically mirrored on the aircraft.
-- To move both FRONT wings in the same physical direction:
--   left raw angle  = -logical angle
--   right raw angle =  logical angle
-- Positive logical angle means the FRONT/leading edge of BOTH wings goes UP.
local function setSynced(logicalAngle)
    left.setTargetAngle(-logicalAngle)
    right.setTargetAngle(logicalAngle)
end

local sequence = {
    { angle = 0,  wait = 1.5 },
    { angle = 5,  wait = 2.0 },
    { angle = 0,  wait = 1.5 },
    { angle = -5, wait = 2.0 },
    { angle = 0,  wait = 1.5 },
}

print("PHYSICAL wing sync test")
print("LEFT : tilt_adapter_1 = -angle")
print("RIGHT: tilt_adapter_2 = +angle")
print("")
print("+5 should make BOTH leading edges go UP.")
print("-5 should make BOTH leading edges go DOWN.")
print("")

for _, step in ipairs(sequence) do
    print("Logical wing angle -> " .. step.angle .. " deg")
    setSynced(step.angle)
    sleep(step.wait)
end

setSynced(0)
print("Done. Both returned to neutral.")
