local left = peripheral.wrap("tilt_adapter_1")
local right = peripheral.wrap("tilt_adapter_2")

assert(left, "Missing tilt_adapter_1 (front left)")
assert(right, "Missing tilt_adapter_2 (front right)")

local sequence = {
    { angle = 0,  wait = 1.5 },
    { angle = 5,  wait = 2.0 },
    { angle = 0,  wait = 1.5 },
    { angle = -5, wait = 2.0 },
    { angle = 0,  wait = 1.5 },
}

print("Tilt sync test")
print("LEFT : tilt_adapter_1")
print("RIGHT: tilt_adapter_2")
print("Both adapters will receive the SAME target angle.")
print("Watch whether both front wings move together.")
print("")

for _, step in ipairs(sequence) do
    print("Both -> " .. step.angle .. " deg")
    left.setTargetAngle(step.angle)
    right.setTargetAngle(step.angle)
    sleep(step.wait)
end

left.setTargetAngle(0)
right.setTargetAngle(0)
print("Done. Both returned to 0 deg.")
