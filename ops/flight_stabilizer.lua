-- Crystalite UAV world-level + heading stabilizer
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
-- Targets:
--   pitch = world level
--   heading = launch/activation heading
--   roll = whatever bank is needed to recover heading, otherwise level
--
-- IMPORTANT CONTROLLER DESIGN:
--   There are NO fixed-angle pitch recovery commands anymore.
--   Pitch correction is continuous: the worse the attitude/rate, the stronger
--   the command; as the aircraft recovers, the command automatically fades.
--   A hysteresis latch only decides when pitch gets PRIORITY over heading/roll,
--   so it cannot flicker recovery on/off around one threshold.

local LEFT_NAME  = "tilt_adapter_1"
local RIGHT_NAME = "tilt_adapter_2"

-- Base pitch PD gains near level.
local PITCH_KP_MIN = 0.72
local PITCH_KD_MIN = 0.18

-- Gains at severe pitch error. Interpolated continuously between MIN and MAX.
local PITCH_KP_MAX = 1.55
local PITCH_KD_MAX = 0.42
local FULL_PITCH_GAIN_AT = 35.0 -- degrees from level

-- Roll controller.
local ROLL_KP  = 0.55
local ROLL_KD  = 0.16
local ROLL_SIGN = 1

-- Heading -> desired bank.
local HEADING_TO_BANK = 0.60
local MAX_BANK_TARGET = 25.0
local HEADING_DEADBAND = 1.0

-- Logical wing-incidence limits.
-- Negative = leading edges DOWN = confirmed anti-backflip direction.
local MIN_COLLECTIVE = -20.0
local MAX_COLLECTIVE =  14.0
local MAX_DIFFERENTIAL = 10.0
local MAX_RAW_ADAPTER = 30.0
local TRIM = 0.0

-- Pitch-priority hysteresis.
-- Enter at a clearly bad attitude/rate, but do not leave until both are genuinely
-- calm. This prevents the old adjust -> cancel -> adjust oscillation.
local PRIORITY_ENTER_PITCH = 10.0 -- absolute degrees
local PRIORITY_ENTER_RATE  = 16.0 -- absolute deg/s
local PRIORITY_EXIT_PITCH  = 3.0
local PRIORITY_EXIT_RATE   = 5.0

-- While pitch priority is active we still allow a little differential authority
-- to keep a huge bank from developing, but heading recovery is suppressed.
local PRIORITY_MAX_DIFFERENTIAL = 3.0

-- Pitch/roll rates are derived from attitude samples. Smooth them because CC tick
-- timing and Sable updates are not perfectly uniform.
local RATE_FILTER_ALPHA = 0.28

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

local function lerp(a,b,t)
    return a + (b-a)*t
end

local function smoothstep01(t)
    t=clamp(t,0,1)
    return t*t*(3-2*t)
end

local function vec(x,y,z) return {x=x,y=y,z=z} end
local function dot(a,b) return a.x*b.x + a.y*b.y + a.z*b.z end
local function cross(a,b)
    return vec(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
end
local function len(v) return math.sqrt(dot(v,v)) end
local function norm(v)
    local l=len(v)
    if l < 1e-9 then return vec(0,0,0) end
    return vec(v.x/l,v.y/l,v.z/l)
end

local function qnorm(q)
    local l=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    assert(l>1e-9,"Invalid zero quaternion from Sable")
    return {x=q.x/l,y=q.y/l,z=q.z/l,w=q.w/l}
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
    assert(type(x)=="number" and type(y)=="number" and type(z)=="number","Bad Sable vector")
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
    assert(pose and pose.orientation,"Sable pose has no orientation")
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

local function setControls(collective,differential,maxDifferential)
    collective=clamp(collective,MIN_COLLECTIVE,MAX_COLLECTIVE)
    local diffLimit=maxDifferential or MAX_DIFFERENTIAL
    differential=clamp(differential,-diffLimit,diffLimit)

    local rawLeft  = -(collective + differential)
    local rawRight =  (collective - differential)
    setRaw(rawLeft,rawRight)
    return rawLeft,rawRight,differential
end

-- Find which LOCAL horizontal axis is the physical nose. The local nose axis is
-- fixed by construction; its WORLD direction is recomputed every single tick.
local function detectForward()
    term.clear(); term.setCursorPos(1,1)
    print("UAV stabilizer")
    print("Targets: LEVEL + HEADING HOLD")
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

local function horizontal(v)
    return norm(vec(v.x,0,v.z))
end

local function headingError(currentForward,targetForward)
    local c=horizontal(currentForward)
    local t=horizontal(targetForward)
    if len(c)<1e-6 or len(t)<1e-6 then return 0 end
    return math.atan(dot(cross(c,t),WORLD_UP),dot(c,t))*RAD2DEG
end

local function headingDeg(forwardWorld)
    local f=horizontal(forwardWorld)
    if len(f)<1e-6 then return 0 end
    return math.atan(f.x,f.z)*RAD2DEG
end

local function attitude(q,forwardLocal)
    -- Dynamic WORLD forward direction: updated from Sable on every loop.
    local forwardWorld=qrotate(q,forwardLocal)
    local upWorld=qrotate(q,vec(0,1,0))

    local h=math.sqrt(forwardWorld.x*forwardWorld.x+forwardWorld.z*forwardWorld.z)
    local pitch=math.atan(forwardWorld.y,h)*RAD2DEG

    local fHoriz=horizontal(forwardWorld)
    local rightHoriz
    if len(fHoriz)<1e-6 then rightHoriz=vec(1,0,0)
    else rightHoriz=norm(cross(fHoriz,WORLD_UP)) end

    local roll=math.atan(dot(upWorld,rightHoriz),dot(upWorld,WORLD_UP))*RAD2DEG
    return pitch,roll,upWorld.y,forwardWorld
end

local function main()
    local forwardLocal,forwardLabel=detectForward()

    -- Capture only horizontal heading; level remains absolute world level.
    local q0=getOrientation()
    local targetForward=horizontal(qrotate(q0,forwardLocal))
    assert(len(targetForward)>1e-6,"Cannot establish heading while nose is vertical")

    local prevPitch,prevRoll=nil,nil
    local filtPitchRate,filtRollRate=0,0
    local pitchPriority=false
    local nextTelemetry=os.clock()
    local lastSample=os.clock()

    while true do
        local now=os.clock()
        local dt=now-lastSample
        lastSample=now
        if dt < 0.01 or dt > 0.25 then dt=LOOP_DT end

        local q=getOrientation()
        local pitch,roll,upY,currentForward=attitude(q,forwardLocal)
        local yawErr=headingError(currentForward,targetForward)

        local rawPitchRate,rawRollRate=0,0
        if prevPitch~=nil then
            rawPitchRate=(pitch-prevPitch)/dt
            rawRollRate=(roll-prevRoll)/dt
        end
        prevPitch,prevRoll=pitch,roll

        filtPitchRate=filtPitchRate + RATE_FILTER_ALPHA*(rawPitchRate-filtPitchRate)
        filtRollRate=filtRollRate + RATE_FILTER_ALPHA*(rawRollRate-filtRollRate)

        local absPitch=math.abs(pitch)
        local absPitchRate=math.abs(filtPitchRate)

        -- Hysteresis: one stable decision, not a threshold that chatters every tick.
        if not pitchPriority then
            if absPitch >= PRIORITY_ENTER_PITCH or absPitchRate >= PRIORITY_ENTER_RATE then
                pitchPriority=true
            end
        else
            if absPitch <= PRIORITY_EXIT_PITCH and absPitchRate <= PRIORITY_EXIT_RATE and upY > 0.35 then
                pitchPriority=false
            end
        end

        -- Continuous gain scheduling. Severe error gets strong authority, but the
        -- command automatically fades as the nose approaches level.
        local severity=smoothstep01(absPitch/FULL_PITCH_GAIN_AT)
        local pitchKp=lerp(PITCH_KP_MIN,PITCH_KP_MAX,severity)
        local pitchKd=lerp(PITCH_KD_MIN,PITCH_KD_MAX,severity)
        local collective=TRIM - pitchKp*pitch - pitchKd*filtPitchRate
        collective=clamp(collective,MIN_COLLECTIVE,MAX_COLLECTIVE)

        local desiredRoll=0
        local mode="NORMAL"

        if pitchPriority then
            -- Do not chase heading while diving/backflipping. First recover the
            -- longitudinal attitude. We still try to stop a large roll.
            mode = pitch >= 0 and "PITCH RECOVERY UP" or "PITCH RECOVERY DOWN"
            desiredRoll=0
        elseif math.abs(yawErr) > HEADING_DEADBAND then
            mode="HEADING RECOVERY"
            desiredRoll=clamp(HEADING_TO_BANK*yawErr,-MAX_BANK_TARGET,MAX_BANK_TARGET)
        end

        local rollError=roll-desiredRoll
        local differential=ROLL_SIGN*(-ROLL_KP*rollError-ROLL_KD*filtRollRate)

        local diffLimit=pitchPriority and PRIORITY_MAX_DIFFERENTIAL or MAX_DIFFERENTIAL
        local rawLeft,rawRight,usedDifferential=setControls(collective,differential,diffLimit)

        if os.clock() >= nextTelemetry then
            nextTelemetry=os.clock()+TELEMETRY_PERIOD
            term.clear(); term.setCursorPos(1,1)
            print("UAV CONTINUOUS ATTITUDE HOLD")
            print("MODE: "..mode)
            print("Body nose: "..forwardLabel)
            print(string.format("Heading %6.1f err %6.1f",headingDeg(currentForward),yawErr))
            print(string.format("Pitch %7.2f rate %7.2f",pitch,filtPitchRate))
            print(string.format("Roll  %7.2f target %6.1f",roll,desiredRoll))
            print(string.format("Severity %.2f  priority %s",severity,pitchPriority and "YES" or "no"))
            print(string.format("Collective %7.2f",collective))
            print(string.format("Differential %5.2f",usedDifferential))
            print(string.format("Raw L/R %6.1f / %6.1f",rawLeft,rawRight))
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
