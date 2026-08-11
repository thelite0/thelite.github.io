-- Crystalite UAV damped attitude + heading stabilizer
-- CC:Sable + Create Propulsion Tilt Adapters
--
-- tilt_adapter_1 = front LEFT
-- tilt_adapter_2 = front RIGHT
-- Positive logical incidence = leading/front edge UP, trailing/rear edge DOWN.
-- Raw adapter signs are mirrored: left=-logical, right=+logical.
--
-- Flight test established:
--   positive collective -> nose-up authority
--   negative collective (manual S direction) -> nose-down / anti-backflip authority
--
-- v5.1: same damped/autotrim controller as v5, but all tuning lives in one CFG
-- table so CC:Tweaked/Lua does not hit the 60-upvalue limit in main().

local CFG = {
    LEFT_NAME = "tilt_adapter_1",
    RIGHT_NAME = "tilt_adapter_2",

    PITCH_KP_NEAR = 0.24,
    PITCH_KP_FAR = 0.58,
    PITCH_KD = 0.34,
    PITCH_GAIN_BLEND_START = 4.0,
    PITCH_GAIN_BLEND_FULL = 20.0,

    ROLL_KP_NEAR = 0.28,
    ROLL_KP_FAR = 0.52,
    ROLL_KD = 0.30,
    ROLL_ACTUATOR_SIGN = 1,

    HEADING_BANK_SIGN = -1,
    HEADING_TO_BANK = 0.18,
    MAX_HEADING_BANK = 8.0,
    HEADING_DEADBAND = 3.0,
    BANK_TARGET_SLEW_DPS = 10.0,
    BANK_TARGET_RETURN_DPS = 24.0,

    MIN_COLLECTIVE = -20.0,
    MAX_COLLECTIVE = 14.0,
    MAX_DIFFERENTIAL = 12.0,
    MAX_RAW_ADAPTER = 30.0,
    BASE_TRIM = 0.0,

    AUTOTRIM_KI = 0.030,
    MAX_AUTOTRIM = 4.0,
    TRIM_SETTLE_TIME = 1.5,
    TRIM_PITCH_WINDOW = 7.0,
    TRIM_ROLL_WINDOW = 7.0,
    TRIM_PITCH_RATE_WINDOW = 7.0,
    TRIM_ROLL_RATE_WINDOW = 8.0,
    TRIM_HEADING_ERROR_WINDOW = 8.0,
    MIN_TRIM_SPEED = 0.70,

    BACKFLIP_PITCH_START = 8.0,
    BACKFLIP_PITCH_FULL = 30.0,
    BACKFLIP_RATE_START = 14.0,
    BACKFLIP_RATE_FULL = 50.0,
    BACKFLIP_MAX_COMMAND = -20.0,

    DIVE_PITCH_START = 10.0,
    DIVE_PITCH_FULL = 38.0,
    DIVE_RATE_START = 16.0,
    DIVE_RATE_FULL = 55.0,
    DIVE_MAX_COMMAND = 14.0,

    HEADING_PITCH_SAFE = 5.0,
    HEADING_PITCH_OFF = 16.0,
    HEADING_ROLL_SAFE = 4.0,
    HEADING_ROLL_OFF = 14.0,
    HEADING_PITCH_RATE_SAFE = 5.0,
    HEADING_PITCH_RATE_OFF = 16.0,
    HEADING_ROLL_RATE_SAFE = 5.0,
    HEADING_ROLL_RATE_OFF = 18.0,
    HEADING_YAW_RATE_SAFE = 7.0,
    HEADING_YAW_RATE_OFF = 24.0,
    MIN_HEADING_SPEED = 0.65,

    COLLECTIVE_SLEW_CALM_DPS = 28.0,
    COLLECTIVE_SLEW_EMERGENCY_DPS = 90.0,
    DIFFERENTIAL_SLEW_CALM_DPS = 24.0,
    DIFFERENTIAL_SLEW_EMERGENCY_DPS = 70.0,

    LOOP_DT = 0.05,
    MIN_DETECT_SPEED = 0.35,
    TELEMETRY_PERIOD = 0.20,
    RATE_ALPHA = 0.22,
}

local left = peripheral.wrap(CFG.LEFT_NAME)
local right = peripheral.wrap(CFG.RIGHT_NAME)
assert(left, "Missing " .. CFG.LEFT_NAME)
assert(right, "Missing " .. CFG.RIGHT_NAME)
assert(type(left.setTargetAngle) == "function", CFG.LEFT_NAME .. " has no setTargetAngle()")
assert(type(right.setTargetAngle) == "function", CFG.RIGHT_NAME .. " has no setTargetAngle()")
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

local function lerp(a, b, t)
    return a + (b - a) * clamp(t, 0, 1)
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
    l=clamp(l,-CFG.MAX_RAW_ADAPTER,CFG.MAX_RAW_ADAPTER)
    r=clamp(r,-CFG.MAX_RAW_ADAPTER,CFG.MAX_RAW_ADAPTER)
    left.setTargetAngle(l)
    right.setTargetAngle(r)
end

local function setControls(collective,differential)
    collective=clamp(collective,CFG.MIN_COLLECTIVE,CFG.MAX_COLLECTIVE)
    differential=clamp(differential,-CFG.MAX_DIFFERENTIAL,CFG.MAX_DIFFERENTIAL)
    local rawLeft=-(collective+differential)
    local rawRight=(collective-differential)
    setRaw(rawLeft,rawRight)
    return rawLeft,rawRight
end

local function detectForward()
    term.clear(); term.setCursorPos(1,1)
    print("UAV stabilizer v5.1")
    print("DAMPED + AUTO-TRIM")
    print("Waiting for forward movement...")
    while true do
        setControls(CFG.BASE_TRIM,0)
        local q=getOrientation()
        local vLocal=qrotate(qconj(q),getVelocity())
        local h=math.sqrt(vLocal.x*vLocal.x+vLocal.z*vLocal.z)
        if h >= CFG.MIN_DETECT_SPEED then
            if math.abs(vLocal.x) >= math.abs(vLocal.z) then
                local s=vLocal.x>=0 and 1 or -1
                return vec(s,0,0), "X"..(s>0 and "+" or "-")
            else
                local s=vLocal.z>=0 and 1 or -1
                return vec(0,0,s), "Z"..(s>0 and "+" or "-")
            end
        end
        sleep(CFG.LOOP_DT)
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
    local forwardWorld=qrotate(q,forwardLocal)
    local upWorld=qrotate(q,vec(0,1,0))
    local rightLocal=norm(cross(forwardLocal,vec(0,1,0)))
    local rightWorld=qrotate(q,rightLocal)
    local h=math.sqrt(forwardWorld.x*forwardWorld.x+forwardWorld.z*forwardWorld.z)
    local pitch=math.atan(forwardWorld.y,h)*RAD2DEG
    local fHoriz=horizontal(forwardWorld)
    local rightHoriz
    if len(fHoriz)<1e-6 then rightHoriz=horizontal(rightWorld)
    else rightHoriz=norm(cross(fHoriz,WORLD_UP)) end
    local roll=math.atan(dot(upWorld,rightHoriz),dot(upWorld,WORLD_UP))*RAD2DEG
    return pitch,roll,upWorld.y,forwardWorld,rightWorld
end

local function angularVelocityWorld(qPrev,qNow,dt)
    local dq=qnorm(qmul(qNow,qconj(qPrev)))
    if dq.w < 0 then dq={x=-dq.x,y=-dq.y,z=-dq.z,w=-dq.w} end
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
    local filteredPitchRate,filteredRollRate,filteredYawRate=0,0,0
    local collectiveCmd=CFG.BASE_TRIM
    local differentialCmd=0
    local desiredRollCmd=0
    local autoTrim=0
    local trimStableTime=0
    local nextTelemetry=os.clock()

    while true do
        local now=os.clock()
        local dt=now-lastTime
        if dt < 0.005 or dt > 0.25 then dt=CFG.LOOP_DT end
        lastTime=now

        q=getOrientation()
        local pitch,roll,upY,currentForward,rightWorld=attitude(q,forwardLocal)
        local speed=len(getVelocity())
        local yawErr=headingError(currentForward,targetForward)

        local omega=angularVelocityWorld(qPrev,q,dt)
        qPrev=q
        local pitchRate=dot(omega,rightWorld)*RAD2DEG
        local rollRate=dot(omega,currentForward)*RAD2DEG
        local yawRate=dot(omega,WORLD_UP)*RAD2DEG

        filteredPitchRate=filteredPitchRate+(pitchRate-filteredPitchRate)*CFG.RATE_ALPHA
        filteredRollRate=filteredRollRate+(rollRate-filteredRollRate)*CFG.RATE_ALPHA
        filteredYawRate=filteredYawRate+(yawRate-filteredYawRate)*CFG.RATE_ALPHA

        local backflipSeverity=math.max(
            smoothstep(CFG.BACKFLIP_PITCH_START,CFG.BACKFLIP_PITCH_FULL,pitch),
            smoothstep(CFG.BACKFLIP_RATE_START,CFG.BACKFLIP_RATE_FULL,filteredPitchRate)
        )
        local diveSeverity=math.max(
            smoothstep(CFG.DIVE_PITCH_START,CFG.DIVE_PITCH_FULL,-pitch),
            smoothstep(CFG.DIVE_RATE_START,CFG.DIVE_RATE_FULL,-filteredPitchRate)
        )
        local pitchSeverity=math.max(backflipSeverity,diveSeverity)

        local rawHeadingBank=0
        if math.abs(yawErr)>CFG.HEADING_DEADBAND then
            rawHeadingBank=CFG.HEADING_BANK_SIGN*CFG.HEADING_TO_BANK*yawErr
            rawHeadingBank=clamp(rawHeadingBank,-CFG.MAX_HEADING_BANK,CFG.MAX_HEADING_BANK)
        end

        local pitchSafety=1-smoothstep(CFG.HEADING_PITCH_SAFE,CFG.HEADING_PITCH_OFF,math.abs(pitch))
        local rollSafety=1-smoothstep(CFG.HEADING_ROLL_SAFE,CFG.HEADING_ROLL_OFF,math.abs(roll))
        local pitchRateSafety=1-smoothstep(CFG.HEADING_PITCH_RATE_SAFE,CFG.HEADING_PITCH_RATE_OFF,math.abs(filteredPitchRate))
        local rollRateSafety=1-smoothstep(CFG.HEADING_ROLL_RATE_SAFE,CFG.HEADING_ROLL_RATE_OFF,math.abs(filteredRollRate))
        local yawRateSafety=1-smoothstep(CFG.HEADING_YAW_RATE_SAFE,CFG.HEADING_YAW_RATE_OFF,math.abs(filteredYawRate))
        local speedSafety=smoothstep(CFG.MIN_HEADING_SPEED,CFG.MIN_HEADING_SPEED+0.8,speed)
        local uprightSafety=smoothstep(0.15,0.70,upY)
        local headingAuthority=clamp(pitchSafety*rollSafety*pitchRateSafety*rollRateSafety*yawRateSafety*speedSafety*uprightSafety,0,1)

        local desiredRollTarget=rawHeadingBank*headingAuthority
        local bankSlew=(headingAuthority < 0.15) and CFG.BANK_TARGET_RETURN_DPS or CFG.BANK_TARGET_SLEW_DPS
        desiredRollCmd=slew(desiredRollCmd,desiredRollTarget,bankSlew,dt)

        local trimEligible =
            speed >= CFG.MIN_TRIM_SPEED and
            upY > 0.75 and
            math.abs(pitch) <= CFG.TRIM_PITCH_WINDOW and
            math.abs(roll) <= CFG.TRIM_ROLL_WINDOW and
            math.abs(filteredPitchRate) <= CFG.TRIM_PITCH_RATE_WINDOW and
            math.abs(filteredRollRate) <= CFG.TRIM_ROLL_RATE_WINDOW and
            math.abs(yawErr) <= CFG.TRIM_HEADING_ERROR_WINDOW and
            math.abs(desiredRollCmd) <= 2.0 and
            pitchSeverity < 0.05

        if trimEligible then
            trimStableTime=trimStableTime+dt
            if trimStableTime >= CFG.TRIM_SETTLE_TIME then
                autoTrim=clamp(autoTrim+CFG.AUTOTRIM_KI*(-pitch)*dt,-CFG.MAX_AUTOTRIM,CFG.MAX_AUTOTRIM)
            end
        else
            trimStableTime=0
        end

        local pitchBlend=smoothstep(CFG.PITCH_GAIN_BLEND_START,CFG.PITCH_GAIN_BLEND_FULL,math.abs(pitch))
        local pitchKp=lerp(CFG.PITCH_KP_NEAR,CFG.PITCH_KP_FAR,pitchBlend)
        local collectiveTarget=CFG.BASE_TRIM+autoTrim-pitchKp*pitch-CFG.PITCH_KD*filteredPitchRate

        local backflipAssist=CFG.BACKFLIP_MAX_COMMAND*backflipSeverity
        if backflipAssist < collectiveTarget then collectiveTarget=backflipAssist end
        local diveAssist=CFG.DIVE_MAX_COMMAND*diveSeverity
        if diveAssist > collectiveTarget then collectiveTarget=diveAssist end
        collectiveTarget=clamp(collectiveTarget,CFG.MIN_COLLECTIVE,CFG.MAX_COLLECTIVE)

        local rollError=roll-desiredRollCmd
        local rollBlend=smoothstep(CFG.ROLL_GAIN_BLEND_START,CFG.ROLL_GAIN_BLEND_FULL,math.abs(rollError))
        local rollKp=lerp(CFG.ROLL_KP_NEAR,CFG.ROLL_KP_FAR,rollBlend)
        local differentialTarget=CFG.ROLL_ACTUATOR_SIGN*(-rollKp*rollError-CFG.ROLL_KD*filteredRollRate)
        differentialTarget=clamp(differentialTarget,-CFG.MAX_DIFFERENTIAL,CFG.MAX_DIFFERENTIAL)

        local collectiveSlew=lerp(CFG.COLLECTIVE_SLEW_CALM_DPS,CFG.COLLECTIVE_SLEW_EMERGENCY_DPS,pitchSeverity)
        local rollSeverity=math.max(
            smoothstep(5.0,24.0,math.abs(rollError)),
            smoothstep(7.0,30.0,math.abs(filteredRollRate))
        )
        local differentialSlew=lerp(CFG.DIFFERENTIAL_SLEW_CALM_DPS,CFG.DIFFERENTIAL_SLEW_EMERGENCY_DPS,rollSeverity)

        collectiveCmd=slew(collectiveCmd,collectiveTarget,collectiveSlew,dt)
        differentialCmd=slew(differentialCmd,differentialTarget,differentialSlew,dt)
        local rawLeft,rawRight=setControls(collectiveCmd,differentialCmd)

        if os.clock()>=nextTelemetry then
            nextTelemetry=os.clock()+CFG.TELEMETRY_PERIOD
            local mode="TRIM / LEVEL"
            if pitchSeverity>0.15 then mode="PITCH RECOVERY"
            elseif math.abs(roll)>CFG.HEADING_ROLL_SAFE or math.abs(filteredRollRate)>CFG.HEADING_ROLL_RATE_SAFE then mode="ROLL DAMPING"
            elseif headingAuthority>0.15 and math.abs(yawErr)>CFG.HEADING_DEADBAND then mode="HEADING HOLD" end

            term.clear(); term.setCursorPos(1,1)
            print("UAV STABILIZER v5.1")
            print("MODE: "..mode)
            print("Body nose: "..forwardLabel)
            print(string.format("Heading %6.1f err %6.1f",headingDeg(currentForward),yawErr))
            print(string.format("Heading auth %.2f bank %5.1f",headingAuthority,desiredRollCmd))
            print(string.format("Pitch %6.1f rate %7.1f",pitch,filteredPitchRate))
            print(string.format("Roll  %6.1f rate %7.1f",roll,filteredRollRate))
            print(string.format("AutoTrim %+5.2f settle %.1fs",autoTrim,trimStableTime))
            print(string.format("Collect %6.1f -> %6.1f",collectiveCmd,collectiveTarget))
            print(string.format("Diff    %6.1f -> %6.1f",differentialCmd,differentialTarget))
            print(string.format("Raw L/R %6.1f / %6.1f",rawLeft,rawRight))
            print("")
            print("Ctrl+T = stop + neutral")
        end

        sleep(CFG.LOOP_DT)
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