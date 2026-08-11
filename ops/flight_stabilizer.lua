-- Crystalite UAV world-level attitude stabilizer prototype
-- CC:Sable + Create Propulsion Tilt Adapters
--
-- tilt_adapter_1 = front LEFT
-- tilt_adapter_2 = front RIGHT
-- Physical convention discovered in testing:
--   positive logical incidence = leading/front edge UP, trailing/rear edge DOWN
--   left raw adapter angle must be inverted, right raw adapter angle is normal.
--
-- IMPORTANT: "straight" is WORLD LEVEL now.
-- Startup attitude is NOT captured as the target.
-- Target pitch = 0 degrees relative to world horizontal.
-- Target roll  = 0 degrees relative to world up.
-- Yaw/heading is intentionally left free.

local LEFT_NAME  = "tilt_adapter_1"
local RIGHT_NAME = "tilt_adapter_2"

-- Conservative first-flight gains. Tune after observing telemetry.
local PITCH_KP = 0.55
local PITCH_KD = 0.16
local ROLL_KP  = 0.45
local ROLL_KD  = 0.12

-- Change to -1 only if roll correction makes a bank WORSE instead of better.
local ROLL_SIGN = 1

-- Degrees of logical wing incidence.
local TRIM = 0.0
local MAX_COLLECTIVE = 8.0
local MAX_DIFFERENTIAL = 6.0
local MAX_RAW_ADAPTER = 10.0

local LOOP_DT = 0.05
local MIN_DETECT_SPEED = 0.35
local TELEMETRY_PERIOD = 0.25

local left = peripheral.wrap(LEFT_NAME)
local right = peripheral.wrap(RIGHT_NAME)

assert(left,  "Missing " .. LEFT_NAME)
assert(right, "Missing " .. RIGHT_NAME)
assert(type(left.setTargetAngle) == "function", LEFT_NAME .. " has no setTargetAngle()")
assert(type(right.setTargetAngle) == "function", RIGHT_NAME .. " has no setTargetAngle()")
assert(sublevel and sublevel.getLogicalPose, "CC:Sable 'sublevel' API is unavailable")
assert(sublevel.getLinearVelocity, "CC:Sable getLinearVelocity() is unavailable")

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function vec(x, y, z)
    return { x = x, y = y, z = z }
end

local function dot(a, b)
    return a.x*b.x + a.y*b.y + a.z*b.z
end

local function cross(a, b)
    return vec(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    )
end

local function length(v)
    return math.sqrt(dot(v, v))
end

local function normalize(v)
    local l = length(v)
    if l < 1e-9 then return vec(0, 0, 0) end
    return vec(v.x/l, v.y/l, v.z/l)
end

local function qnorm(q)
    local l = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    assert(l > 1e-9, "Invalid zero quaternion from Sable")
    return { x=q.x/l, y=q.y/l, z=q.z/l, w=q.w/l }
end

local function qconj(q)
    return { x=-q.x, y=-q.y, z=-q.z, w=q.w }
end

local function qmul(a, b)
    return {
        w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    }
end

local function qrotate(q, v)
    local p = { x=v.x, y=v.y, z=v.z, w=0 }
    local r = qmul(qmul(q, p), qconj(q))
    return vec(r.x, r.y, r.z)
end

-- Installed CC:Sable returns orientation as axis-angle:
--   { a = angleRadians, v = { axisX, axisY, axisZ } }
-- Support quaternion format too in case the API changes.
local function tableXYZ(t)
    assert(type(t) == "table", "Expected vector table")

    local x = t.x
    local y = t.y
    local z = t.z

    if x == nil then x = t[1] end
    if y == nil then y = t[2] end
    if z == nil then z = t[3] end

    assert(type(x) == "number" and type(y) == "number" and type(z) == "number",
        "Unsupported Sable vector format")

    return x, y, z
end

local function orientationToQuaternion(o)
    assert(type(o) == "table", "Sable orientation is not a table")

    if type(o.x) == "number" and type(o.y) == "number" and
       type(o.z) == "number" and type(o.w) == "number" then
        return qnorm({x=o.x, y=o.y, z=o.z, w=o.w})
    end

    if type(o.a) == "number" and type(o.v) == "table" then
        local ax, ay, az = tableXYZ(o.v)
        local axisLen = math.sqrt(ax*ax + ay*ay + az*az)

        if axisLen < 1e-9 then
            if math.abs(o.a) < 1e-9 then
                return {x=0, y=0, z=0, w=1}
            end
            error("Sable returned non-zero axis-angle rotation with zero axis")
        end

        ax, ay, az = ax/axisLen, ay/axisLen, az/axisLen
        local half = o.a * 0.5
        local s = math.sin(half)

        return qnorm({
            x = ax * s,
            y = ay * s,
            z = az * s,
            w = math.cos(half)
        })
    end

    error("Unsupported CC:Sable orientation format")
end

local RAD2DEG = 180 / math.pi
local WORLD_UP = vec(0, 1, 0)

local function setRaw(leftAngle, rightAngle)
    leftAngle = clamp(leftAngle, -MAX_RAW_ADAPTER, MAX_RAW_ADAPTER)
    rightAngle = clamp(rightAngle, -MAX_RAW_ADAPTER, MAX_RAW_ADAPTER)
    left.setTargetAngle(leftAngle)
    right.setTargetAngle(rightAngle)
end

local function setControls(collective, differential)
    collective = clamp(collective, -MAX_COLLECTIVE, MAX_COLLECTIVE)
    differential = clamp(differential, -MAX_DIFFERENTIAL, MAX_DIFFERENTIAL)

    -- Logical physical incidence:
    --   positive collective = BOTH leading edges up.
    -- Raw adapter signs are mirrored on the airframe.
    local rawLeft  = -(collective + differential)
    local rawRight =  (collective - differential)

    setRaw(rawLeft, rawRight)
    return rawLeft, rawRight
end

local function getOrientation()
    local pose = sublevel.getLogicalPose()
    assert(pose and pose.orientation, "Sable pose has no orientation")
    return orientationToQuaternion(pose.orientation)
end

local function getVelocity()
    local v = sublevel.getLinearVelocity()
    assert(v and v.x ~= nil and v.y ~= nil and v.z ~= nil, "Sable returned invalid linear velocity")
    return vec(v.x, v.y, v.z)
end

local function detectForward()
    term.clear()
    term.setCursorPos(1, 1)
    print("UAV stabilizer")
    print("Target: WORLD LEVEL")
    print("Pitch target: 0 deg")
    print("Roll target : 0 deg")
    print("Waiting for forward movement...")

    while true do
        setControls(TRIM, 0)

        local qNow = getOrientation()
        local vWorld = getVelocity()

        -- Convert velocity into the aircraft's CURRENT body frame.
        -- This does not assume startup attitude was straight.
        local vLocal = qrotate(qconj(qNow), vWorld)
        local bodyHorizontal = math.sqrt(vLocal.x*vLocal.x + vLocal.z*vLocal.z)

        if bodyHorizontal >= MIN_DETECT_SPEED then
            if math.abs(vLocal.x) >= math.abs(vLocal.z) then
                local s = vLocal.x >= 0 and 1 or -1
                return vec(s, 0, 0), "X" .. (s > 0 and "+" or "-")
            else
                local s = vLocal.z >= 0 and 1 or -1
                return vec(0, 0, s), "Z" .. (s > 0 and "+" or "-")
            end
        end

        sleep(LOOP_DT)
    end
end

local function getWorldLevelAttitude(qNow, forwardLocal)
    local upLocal = vec(0, 1, 0)
    local forwardWorld = qrotate(qNow, forwardLocal)
    local upWorld = qrotate(qNow, upLocal)

    -- Pitch is simply nose elevation above/below the WORLD horizon.
    local horizontalLength = math.sqrt(
        forwardWorld.x*forwardWorld.x + forwardWorld.z*forwardWorld.z
    )
    local pitch = math.atan(forwardWorld.y, horizontalLength) * RAD2DEG

    -- Roll is measured against WORLD vertical while preserving whatever yaw
    -- heading the aircraft currently has.
    local forwardHorizontal = normalize(vec(forwardWorld.x, 0, forwardWorld.z))
    local rightHorizontal

    if length(forwardHorizontal) < 1e-6 then
        -- Near a vertical nose attitude yaw is undefined; pick a harmless axis.
        rightHorizontal = vec(1, 0, 0)
    else
        rightHorizontal = normalize(cross(forwardHorizontal, WORLD_UP))
    end

    local roll = math.atan(
        dot(upWorld, rightHorizontal),
        dot(upWorld, WORLD_UP)
    ) * RAD2DEG

    return pitch, roll
end

local function main()
    local forwardLocal, forwardLabel = detectForward()

    local previousPitch = 0
    local previousRoll = 0
    local firstSample = true
    local nextTelemetry = os.clock()

    while true do
        local qNow = getOrientation()
        local pitch, roll = getWorldLevelAttitude(qNow, forwardLocal)

        local pitchRate = 0
        local rollRate = 0
        if not firstSample then
            pitchRate = (pitch - previousPitch) / LOOP_DT
            rollRate = (roll - previousRoll) / LOOP_DT
        end
        firstSample = false
        previousPitch = pitch
        previousRoll = roll

        -- WORLD-LEVEL target is always pitch=0, roll=0.
        -- Negative pitch (nosedive) therefore commands positive collective incidence.
        local collective = TRIM - PITCH_KP*pitch - PITCH_KD*pitchRate

        local differential = ROLL_SIGN * (-ROLL_KP*roll - ROLL_KD*rollRate)

        collective = clamp(collective, -MAX_COLLECTIVE, MAX_COLLECTIVE)
        differential = clamp(differential, -MAX_DIFFERENTIAL, MAX_DIFFERENTIAL)

        local rawLeft, rawRight = setControls(collective, differential)

        if os.clock() >= nextTelemetry then
            nextTelemetry = os.clock() + TELEMETRY_PERIOD
            term.clear()
            term.setCursorPos(1, 1)
            print("UAV WORLD-LEVEL HOLD - ACTIVE")
            print("Target pitch/roll: 0 / 0 deg")
            print("Forward axis: " .. forwardLabel)
            print(string.format("Pitch: %7.2f deg  rate: %7.2f", pitch, pitchRate))
            print(string.format("Roll : %7.2f deg  rate: %7.2f", roll, rollRate))
            print(string.format("Collective: %6.2f deg", collective))
            print(string.format("Differential: %4.2f deg", differential))
            print(string.format("Raw L/R: %6.2f / %6.2f", rawLeft, rawRight))
            print("")
            print("Ctrl+T = stop + neutral wings")
        end

        sleep(LOOP_DT)
    end
end

local ok, err = pcall(main)

-- Fail safe: return both physical wing incidences to neutral.
pcall(function() setRaw(0, 0) end)

term.clear()
term.setCursorPos(1, 1)
if ok then
    print("Stabilizer stopped. Wings neutral.")
else
    print("Stabilizer stopped: " .. tostring(err))
    print("Wings commanded neutral.")
end
