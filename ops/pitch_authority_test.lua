local LEFT_NAME  = "tilt_adapter_1"
local RIGHT_NAME = "tilt_adapter_2"

local left = peripheral.wrap(LEFT_NAME)
local right = peripheral.wrap(RIGHT_NAME)

assert(left, "Missing " .. LEFT_NAME)
assert(right, "Missing " .. RIGHT_NAME)

local TEST_ANGLE = 6

local function setIncidence(angle)
    -- Confirmed mirrored adapter mapping:
    -- positive logical incidence = BOTH leading/front edges UP.
    left.setTargetAngle(-angle)
    right.setTargetAngle(angle)
end

local function draw(angle)
    term.clear()
    term.setCursorPos(1, 1)
    print("PITCH AUTHORITY TEST")
    print("")
    print("W = leading edges UP   (+" .. TEST_ANGLE .. " deg)")
    print("S = leading edges DOWN (-" .. TEST_ANGLE .. " deg)")
    print("X = neutral (0 deg)")
    print("Q = neutral + quit")
    print("")
    print(string.format("Current logical incidence: %+d deg", angle))
    print("")
    print("Use briefly in flight and watch NOSE response.")
    print("We need to know which command produces NOSE-DOWN torque.")
end

local current = 0
setIncidence(current)
draw(current)

local ok, err = pcall(function()
    while true do
        local _, key = os.pullEvent("key")

        if key == keys.w then
            current = TEST_ANGLE
            setIncidence(current)
            draw(current)
        elseif key == keys.s then
            current = -TEST_ANGLE
            setIncidence(current)
            draw(current)
        elseif key == keys.x then
            current = 0
            setIncidence(current)
            draw(current)
        elseif key == keys.q then
            break
        end
    end
end)

pcall(function() setIncidence(0) end)
term.clear()
term.setCursorPos(1, 1)

if ok then
    print("Pitch test stopped. Wings neutral.")
else
    print("Pitch test stopped: " .. tostring(err))
    print("Wings commanded neutral.")
end
