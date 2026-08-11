-- Crystalite UAV world-level attitude stabilizer
-- CC:Sable + Create Propulsion Tilt Adapters
--
-- tilt_adapter_1 = front LEFT
-- tilt_adapter_2 = front RIGHT
-- Positive logical incidence = leading/front edge UP, trailing/rear edge DOWN.
-- Raw adapter signs are mirrored: left=-logical, right=+logical.
--
-- Flight test established:
--   NEGATIVE logical incidence (same direction as manual S key)
--   counters a nose-up/backflip tendency.
--
-- Target attitude: WORLD LEVEL (pitch 0, roll 0). Yaw is free.

local LEFT_NAME  = "tilt_adapter_1"
local RIGHT_NAME = "tilt_adapter_2"

-- Normal controller. Much stronger than the first prototype.
local PITCH_KP = 0.95
local PITCH_KD = 0.28
local ROLL_KP  = 0.45
local ROLL_KD  = 0.12
local ROLL_SIGN = 1

-- Logical wing-incidence limits.
-- Negative = leading edges DOWN = confirmed anti-backflip direction.
local MIN_COLLECTIVE = -20.0
local MAX_COLLECTIVE =  12.0
local MAX_DIFFERENTIAL = 6.0
local MAX_RAW_ADAPTER = 30.0
local TRIM = 0.0

-- Emergency anti-backflip mode.
-- It activates BEFORE the aircraft gets close to vertical, because pitch becomes
-- ambiguous through a full loop and recovery is much harder once inverted.
local BACKFLIP_PITCH_TRIGGER = 7.0       -- deg nose-up
local BACKFLIP_RATE_TRIGGER  = 12.0      -- deg/s nose rising
local BACKFLIP_COMMAND       = -20.0     -- hard S-direction command
local BACKFLIP_RELEASE_PITCH = 2.0       -- leave emergency once nearly level
local BACKFLIP_RELEASE_RATE  = 2.0       -- and no longer rotating up quickly

-- Nosedive recovery is deliberately less aggressive because positive incidence
-- previously caused strong nose-up authority on this testbed.
local DIVE_PITCH_TRIGGER = -12.0
local DIVE_RATE_TRIGGER  = -18.0
local DIVE_COMMAND       = 10.0

local LOOP_DT = 0.05
local MIN_DETECT_SPEED = 0.35
local TELEMETRY_PERIOD = 0.20

local left = peripheral.wrap(LEFT_NAME)
local right = peripheral.wrap(RIGHT_NAME)
assert(left,  "Missing " .. LEFT_NAME)
assert(right, "Missing " .. RIGHT_NAME)
assert(type(left.setTargetAngle) == "function", LEFT_NAME .. " has no setTargetAngle()")
assert(type(right.setTargetAngle) == "function", RIGHT_NAME .. " has no setTargetAngle()")
assert(sublevel and sublevel.getLogicalPose, "CC:Sable sublevel API unavailable")
assert(sublevel.getLinearVelocity, "CC:Sable linear velocity unavailable")

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function vec(x, y, z) return {x=x, y=y, z=z} end
local function dot(a,b) return a.x*b.x + a.y*b.y + a.z*b.z end
local function cross(a,b)
    return vec(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
end
local function len(v) return math.sqrt(dot(v,v)) end
local function norm(v)
    local l = len(v)
    if l < 1e-9 then return vec(0,0,0) end
    return vec(v.x/l, v.y/l, v.z/l)
end

local function qnorm(q)
    local l = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    assert(l > 1e-9, "Invalid zero quaternion from Sable")
    return {x=q.x/l, y=q.y/l, z=q.z/l, w=q.w/l}
end
local function qconj(q) return {x=-q.x,y=-q.y,z=-q.z,w=q.w} end
local function qmul(a,b)
    return {
        w=a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
        x=a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
        y=a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
        z=a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,
    }
end
local function qrotate(q,v)
    local p={x=v.x,y=v.y,z=v.z,w=0}
    local r=qmul(qmul(q,p),qconj(q))
    return vec(r.x,r.y,r.z)
end

local function xyz(t)
    local x=t.x; local y=t.y; local z=t.z
    if x==nil then x=t[1] end
    if y==nil then y=t[2] end
    if z==nil then z=t[3] end
    assert(type(x)=="number" and type(y)=="number" and type(z)=="number", "Bad Sable vector")
    return x,y,z
end

local function orientationToQuaternion(o)
    if type(o.x)=="number" and type(o.y)=="number" and type(o.z)=="number" and type(o.w)=="number" then
        return qnorm({x=o.x,y=o.y,z=o.z,w=o.w})
    end
    if type(o.a)=="number" and type(o.v)=="table" then
        local ax,ay,az=xyz(o.v)
        local l=math.sqrt(ax*ax+ay*ay+az*az)
        if l < 1e-9 then
            if math.abs(o.a) < 1e-9 then return {x=0,y=0,z=0,w=1} end
            error("Sable axis-angle has zero axis")
        end
        ax,ay,az=ax/l,ay/l,az/l
        local h=o.a*0.5
        local s=math.sin(h)
        return qnorm({x=ax*s,y=ay*s,z=az*s,w=math.cos(h)})
    end
    error("Unsupported CC:Sable orientation format")
end

local function getOrientation()
    local pose=sublevel.getLogicalPose()
    assert(pose and pose.orientation, "Sable pose has no orientation")
    return orientationToQuaternion(pose.orientation)
end
local function getVelocity()
    local v=sublevel.getLinearVelocity()
    return vec(v.x,v.y,v.z)
end

local function setRaw(l,r)
    l=clamp(l,-MAX_RAW_ADAPTER,MAX_RAW_ADAPTER)
    r=clamp(r,-MAX_RAW_ADAPTER,MAX_RAW_ADAPTER)
    left.setTargetAngle(l)
    right.setTargetAngle(r)
end

local function setControls(collective,differential)
    collective=clamp(collective,MIN_COLLECTIVE,MAX_COLLECTIVE)
    differential=clamp(differential,-MAX_DIFFERENTIAL,MAX_DIFFERENTIAL)

    -- Physical logical incidence:
    -- left wing  = collective + differential
    -- right wing = collective - differential
    -- mirrored adapter signs convert those to raw commands.
    local rawLeft  = -(collective + differential)
    local rawRight =  (collective - differential)
    setRaw(rawLeft,rawRight)
    return rawLeft,rawRight
end

local function detectForward()
    term.clear(); term.setCursorPos(1,1)
    print("UAV stabilizer")
    print("Target: WORLD LEVEL")
    print("Waiting for forward movement...")

    while true do
        setControls(TRIM,0)
        local q=getOrientation()
        local vLocal=qrotate(qconj(q),getVelocity())
        local h=math.sqrt(vLocal.x*vLocal.x+vLocal.z*vLocal.z)
        if h >= MIN_DETECT_SPEED then
            if math.abs(vLocal.x) >= math.abs(vLocal.z) then
                local s=vLocal.x>=0 and 1 or -1
                return vec(s,0,0), "X"..(s>0 and "+" or "-")
            else
                local s=vLocal.z>=0 and 1 or -1
                return vec(0,0,s), "Z"..(s>0 and "+" or "-")
            end
        end
        sleep(LOOP_DT)
    end
end

local WORLD_UP=vec(0,1,0)
local RAD2DEG=180/math.pi

local function attitude(q,forwardLocal)
    local forwardWorld=qrotate(q,forwardLocal)
    local upWorld=qrotate(q,vec(0,1,0))

    local horizontal=math.sqrt(forwardWorld.x*forwardWorld.x+forwardWorld.z*forwardWorld.z)
    local pitch=math.atan(forwardWorld.y,horizontal)*RAD2DEG

    local fHoriz=norm(vec(forwardWorld.x,0,forwardWorld.z))
    local rightHoriz
    if len(fHoriz)<1e-6 then rightHoriz=vec(1,0,0)
    else rightHoriz=norm(cross(fHoriz,WORLD_UP)) end

    local roll=math.atan(dot(upWorld,rightHoriz),dot(upWorld,WORLD_UP))*RAD2DEG
    return pitch,roll,upWorld.y
end

local function main()
    local forwardLocal,forwardLabel=detectForward()
    local prevPitch,prevRoll=0,0
    local first=true
    local emergency=false
    local nextTelemetry=os.clock()

    while true do
        local pitch,roll,upY=attitude(getOrientation(),forwardLocal)
        local pitchRate,rollRate=0,0
        if not first then
            pitchRate=(pitch-prevPitch)/LOOP_DT
            rollRate=(roll-prevRoll)/LOOP_DT
        end
        first=false
        prevPitch,prevRoll=pitch,roll

        local mode="NORMAL"
        local collective
        local differential

        -- Latch anti-backflip until the pitch and pitch-rate are both sane again.
        if not emergency and (pitch >= BACKFLIP_PITCH_TRIGGER or pitchRate >= BACKFLIP_RATE_TRIGGER) then
            emergency=true
        elseif emergency and pitch <= BACKFLIP_RELEASE_PITCH and pitchRate <= BACKFLIP_RELEASE_RATE and upY > 0.25 then
            emergency=false
        end

        if emergency then
            mode="ANTI-BACKFLIP"
            collective=BACKFLIP_COMMAND
            differential=0 -- pitch recovery gets absolute priority
        elseif pitch <= DIVE_PITCH_TRIGGER or pitchRate <= DIVE_RATE_TRIGGER then
            mode="DIVE RECOVERY"
            collective=DIVE_COMMAND
            differential=0
        else
            collective=TRIM-PITCH_KP*pitch-PITCH_KD*pitchRate
            differential=ROLL_SIGN*(-ROLL_KP*roll-ROLL_KD*rollRate)
        end

        local rawLeft,rawRight=setControls(collective,differential)

        if os.clock() >= nextTelemetry then
            nextTelemetry=os.clock()+TELEMETRY_PERIOD
            term.clear(); term.setCursorPos(1,1)
            print("UAV WORLD-LEVEL HOLD")
            print("MODE: "..mode)
            print("Forward: "..forwardLabel)
            print(string.format("Pitch %6.1f  rate %7.1f",pitch,pitchRate))
            print(string.format("Roll  %6.1f  rate %7.1f",roll,rollRate))
            print(string.format("Collective %6.1f",collective))
            print(string.format("Raw L/R %6.1f / %6.1f",rawLeft,rawRight))
            print(string.format("World up dot: %.2f",upY))
            print("")
            print("Ctrl+T = stop + neutral")
        end

        sleep(LOOP_DT)
    end
end

local ok,err=pcall(main)
pcall(function() setRaw(0,0) end)
term.clear(); term.setCursorPos(1,1)
if ok then print("Stabilizer stopped. Wings neutral.")
else print("Stabilizer stopped: "..tostring(err)); print("Wings neutral.") end
