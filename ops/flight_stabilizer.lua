-- Crystalite UAV robust attitude + heading stabilizer
-- CC:Sable + Create Propulsion Tilt Adapters
--
-- tilt_adapter_1 = front LEFT
-- tilt_adapter_2 = front RIGHT
-- Positive logical incidence = leading/front edge UP, trailing/rear edge DOWN.
-- Raw adapter signs are mirrored: left=-logical, right=+logical.
--
-- Flight test established:
--   NEGATIVE collective (manual S direction) counters nose-up/backflip.
--
-- Control philosophy:
--   * no binary dive/normal mode switching
--   * pitch correction is continuous and fades smoothly near level
--   * roll LEVELING always wins over heading hold when the aircraft is badly tilted
--   * heading hold only asks for a modest bank while attitude is already safe
--   * body forward axis is discovered once, but WORLD forward is recomputed every tick

local LEFT_NAME  = "tilt_adapter_1"
local RIGHT_NAME = "tilt_adapter_2"

-- Pitch: logical collective wing incidence per attitude/rate error.
local PITCH_KP = 0.82
local PITCH_KD = 0.30

-- Roll: differential wing incidence. Keep this at +1 unless a pure roll test
-- proves the differential actuator itself is reversed.
local ROLL_KP = 0.72
local ROLL_KD = 0.22
local ROLL_ACTUATOR_SIGN = 1

-- Heading is recovered by banking, NOT by pretending differential lift is a rudder.
-- IMPORTANT: the previous script had this sign backwards.
local HEADING_BANK_SIGN = -1
local HEADING_TO_BANK = 0.30
local MAX_HEADING_BANK = 12.0
local HEADING_DEADBAND = 2.0

-- Wing command limits.
local MIN_COLLECTIVE = -20.0 -- confirmed anti-backflip direction
local MAX_COLLECTIVE =  14.0
local MAX_DIFFERENTIAL = 12.0
local MAX_RAW_ADAPTER = 30.0
local TRIM = 0.0

-- Smooth emergency assistance. These do NOT create discrete modes.
local BACKFLIP_PITCH_START = 6.0
local BACKFLIP_PITCH_FULL  = 28.0
local BACKFLIP_RATE_START  = 10.0
local BACKFLIP_RATE_FULL   = 45.0
local BACKFLIP_MAX_COMMAND = -20.0

local DIVE_PITCH_START = 8.0
local DIVE_PITCH_FULL  = 35.0
local DIVE_RATE_START  = 12.0
local DIVE_RATE_FULL   = 50.0
local DIVE_MAX_COMMAND = 14.0

-- Heading authority fades away as attitude becomes unsafe.
local HEADING_PITCH_SAFE = 6.0
local HEADING_PITCH_OFF  = 20.0
local HEADING_ROLL_SAFE  = 5.0
local HEADING_ROLL_OFF   = 16.0
local MIN_HEADING_SPEED  = 0.55

-- Smooth command slew. Prevents target angle from snapping/cancelling every tick.
local COLLECTIVE_SLEW_DPS = 80.0
local DIFFERENTIAL_SLEW_DPS = 65.0

local LOOP_DT = 0.05
local MIN_DETECT_SPEED = 0.35
local TELEMETRY_PERIOD = 0.20

local left = peripheral.wrap(LEFT_NAME)
local right = peripheral.wrap(RIGHT_NAME)
assert(left, "Missing " .. LEFT_NAME)
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

local function smoothstep(edge0, edge1, x)
    if edge1 <= edge0 then return x >= edge1 and 1 or 0 end
    local t = clamp((x - edge0) / (edge1 - edge0), 0, 1)
    return t * t * (3 - 2 * t)
end

local function slew(current, target, ratePerSec, dt)
    local maxStep = ratePerSec * dt
    local d = target - current
    if d > maxStep then d = maxStep end
    if d < -maxStep then d = -maxStep end
    return current + d
end

local function vec(x,y,z) return {x=x,y=y,z=z} end
local function dot(a,b) return a.x*b.x + a.y*b.y + a.z*b.z end
local function cross(a,b)
    return vec(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
end
local function len(v) return math.sqrt(dot(v,v)) end
local function norm(v)
    local l = len(v)
    if l < 1e-9 then return vec(0,0,0) end
    return vec(v.x/l,v.y/l,v.z/l)
end

local function qnorm(q)
    local l = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    assert(l > 1e-9, "Invalid zero quaternion from Sable")
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
    assert(type(x)=="number" and type(y)=="number" and type(z)=="number", "Bad Sable vector")
    return x,y,z
end

local function orientationToQuaternion(o)
    assert(type(o)=="table", "Sable orientation is not a table")

    if type(o.x)=="number" and type(o.y)=="number" and type(o.z)=="number" and type(o.w)=="number" then
        return qnorm({x=o.x,y=o.y,z=o.z,w=o.w})
    end

    -- Installed CC:Sable build returns axis-angle: { a=<radians>, v=<axis vector> }.
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
    assert(v and v.x~=nil and v.y~=nil and v.z~=nil, "Bad Sable velocity")
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

    -- logical left/right incidence -> mirrored raw Tilt Adapter targets
    local rawLeft  = -(collective + differential)
    local rawRight =  (collective - differential)
    setRaw(rawLeft,rawRight)
    return rawLeft,rawRight
end

local function detectForward()
    term.clear(); term.setCursorPos(1,1)
    print("UAV stabilizer v4")
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
    -- dynamic WORLD forward, updated every tick from latest Sable orientation
    local forwardWorld=qrotate(q,forwardLocal)
    local upWorld=qrotate(q,vec(0,1,0))
    local rightWorld=qrotate(q,norm(cross(forwardLocal,vec(0,1,0))))

    local h=math.sqrt(forwardWorld.x*forwardWorld.x+forwardWorld.z*forwardWorld.z)
    local pitch=math.atan(forwardWorld.y,h)*RAD2DEG

    local fHoriz=horizontal(forwardWorld)
    local rightHoriz
    if len(fHoriz)<1e-6 then rightHoriz=horizontal(rightWorld)
    else rightHoriz=norm(cross(fHoriz,WORLD_UP)) end

    local roll=math.atan(dot(upWorld,rightHoriz),dot(upWorld,WORLD_UP))*RAD2DEG
    return pitch,roll,upWorld.y,forwardWorld,rightWorld
end

-- Derive angular velocity from consecutive Sable quaternions. This avoids noisy
-- finite-difference pitch/roll angles and keeps the rate sign valid near steep attitudes.
local function angularVelocityWorld(qPrev,qNow,dt)
    local dq=qnorm(qmul(qNow,qconj(qPrev)))

    -- use shortest equivalent quaternion rotation
    if dq.w < 0 then
        dq={x=-dq.x,y=-dq.y,z=-dq.z,w=-dq.w}
    end

    local w=clamp(dq.w,-1,1)
    local angle=2*math.acos(w)
    local s=math.sqrt(math.max(0,1-w*w))
    if s < 1e-7 or dt <= 1e-6 then return vec(0,0,0) end

    local axis=vec(dq.x/s,dq.y/s,dq.z/s)
    return vec(axis.x*angle/dt,axis.y*angle/dt,axis.z*angle/dt)
end

local function main()
    local forwardLocal,forwardLabel=detectForward()

    local q=getOrientation()
    local targetForward=horizontal(qrotate(q,forwardLocal))
    assert(len(targetForward)>1e-6,"Cannot establish heading with vertical nose")

    local qPrev=q
    local lastTime=os.clock()
    local filteredPitchRate=0
    local filteredRollRate=0
    local filteredYawRate=0

    local collectiveCmd=0
    local differentialCmd=0
    local nextTelemetry=os.clock()

    while true do
        local now=os.clock()
        local dt=now-lastTime
        if dt < 0.005 or dt > 0.25 then dt=LOOP_DT end
        lastTime=now

        q=getOrientation()
        local pitch,roll,upY,currentForward,rightWorld=attitude(q,forwardLocal)
        local speed=len(getVelocity())
        local yawErr=headingError(currentForward,targetForward)

        local omega=angularVelocityWorld(qPrev,q,dt)
        qPrev=q

        -- Positive pitch rate = nose rising. Positive roll rate follows roll sign above.
        local pitchRate=dot(omega,rightWorld)*RAD2DEG
        local rollRate=dot(omega,currentForward)*RAD2DEG
        local yawRate=dot(omega,WORLD_UP)*RAD2DEG

        -- Small low-pass filter: enough damping without the old stop/start behavior.
        local RATE_ALPHA=0.35
        filteredPitchRate=filteredPitchRate+(pitchRate-filteredPitchRate)*RATE_ALPHA
        filteredRollRate=filteredRollRate+(rollRate-filteredRollRate)*RATE_ALPHA
        filteredYawRate=filteredYawRate+(yawRate-filteredYawRate)*RATE_ALPHA

        -- ----- PITCH: fully continuous -----
        local collectiveTarget=TRIM-PITCH_KP*pitch-PITCH_KD*filteredPitchRate

        -- Smooth anti-backflip assistance. As the plane recovers, severity fades to 0.
        local backflipSeverity=math.max(
            smoothstep(BACKFLIP_PITCH_START,BACKFLIP_PITCH_FULL,pitch),
            smoothstep(BACKFLIP_RATE_START,BACKFLIP_RATE_FULL,filteredPitchRate)
        )
        local backflipAssist=BACKFLIP_MAX_COMMAND*backflipSeverity
        if backflipAssist < collectiveTarget then collectiveTarget=backflipAssist end

        -- Smooth nosedive assistance.
        local diveSeverity=math.max(
            smoothstep(DIVE_PITCH_START,DIVE_PITCH_FULL,-pitch),
            smoothstep(DIVE_RATE_START,DIVE_RATE_FULL,-filteredPitchRate)
        )
        local diveAssist=DIVE_MAX_COMMAND*diveSeverity
        if diveAssist > collectiveTarget then collectiveTarget=diveAssist end

        collectiveTarget=clamp(collectiveTarget,MIN_COLLECTIVE,MAX_COLLECTIVE)

        -- ----- HEADING -> BANK TARGET -----
        -- Critical fix: previous heading->bank sign was inverted.
        local rawHeadingBank=0
        if math.abs(yawErr)>HEADING_DEADBAND then
            rawHeadingBank=HEADING_BANK_SIGN*HEADING_TO_BANK*yawErr
            rawHeadingBank=clamp(rawHeadingBank,-MAX_HEADING_BANK,MAX_HEADING_BANK)
        end

        -- Gradually remove heading authority if pitch/roll is unsafe.
        -- If the aircraft is visibly tilted, desiredRoll tends to ZERO, so leveling wins.
        local pitchSafety=1-smoothstep(HEADING_PITCH_SAFE,HEADING_PITCH_OFF,math.abs(pitch))
        local rollSafety=1-smoothstep(HEADING_ROLL_SAFE,HEADING_ROLL_OFF,math.abs(roll))
        local speedSafety=smoothstep(MIN_HEADING_SPEED,MIN_HEADING_SPEED+0.8,speed)
        local uprightSafety=smoothstep(0.05,0.55,upY)
        local headingAuthority=clamp(pitchSafety*rollSafety*speedSafety*uprightSafety,0,1)

        local desiredRoll=rawHeadingBank*headingAuthority

        -- ----- ROLL: always active -----
        local rollError=roll-desiredRoll
        local differentialTarget=ROLL_ACTUATOR_SIGN*(-ROLL_KP*rollError-ROLL_KD*filteredRollRate)
        differentialTarget=clamp(differentialTarget,-MAX_DIFFERENTIAL,MAX_DIFFERENTIAL)

        -- Smooth actuator targets. No command/cancel/command snapping.
        collectiveCmd=slew(collectiveCmd,collectiveTarget,COLLECTIVE_SLEW_DPS,dt)
        differentialCmd=slew(differentialCmd,differentialTarget,DIFFERENTIAL_SLEW_DPS,dt)

        local rawLeft,rawRight=setControls(collectiveCmd,differentialCmd)

        if os.clock()>=nextTelemetry then
            nextTelemetry=os.clock()+TELEMETRY_PERIOD

            local mode="LEVEL"
            if backflipSeverity>0.15 or diveSeverity>0.15 then mode="PITCH RECOVERY"
            elseif math.abs(roll)>HEADING_ROLL_SAFE then mode="ROLL RECOVERY"
            elseif headingAuthority>0.1 and math.abs(yawErr)>HEADING_DEADBAND then mode="HEADING HOLD" end

            term.clear(); term.setCursorPos(1,1)
            print("UAV STABILIZER v4")
            print("MODE: "..mode)
            print("Body nose: "..forwardLabel)
            print(string.format("Heading %6.1f err %6.1f",headingDeg(currentForward),yawErr))
            print(string.format("Heading authority %.2f",headingAuthority))
            print(string.format("Pitch %6.1f rate %7.1f",pitch,filteredPitchRate))
            print(string.format("Roll  %6.1f target %6.1f",roll,desiredRoll))
            print(string.format("Roll rate %7.1f",filteredRollRate))
            print(string.format("Collective %6.1f -> %6.1f",collectiveCmd,collectiveTarget))
            print(string.format("Diff       %6.1f -> %6.1f",differentialCmd,differentialTarget))
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
if ok then
    print("Stabilizer stopped. Wings neutral.")
else
    print("Stabilizer stopped: "..tostring(err))
    print("Wings neutral.")
end
