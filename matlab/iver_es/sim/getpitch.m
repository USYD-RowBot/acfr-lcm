function getpitch(counter)
global iver state
lastErrProp = iver.errProp;
iver.errProp = iver.goalAlt - state.altMod(counter);
errDir = iver.errProp - lastErrProp;
iver.errInt = iver.errProp + iver.errInt;

if iver.Ki*iver.errInt < -15
    iver.errInt = -15/iver.Ki;
elseif iver.Ki*iver.errInt > 15
    iver.errInt = 15/iver.Ki;
end

goalPitch = iver.Kp*iver.errProp + iver.Ki*iver.errInt + ...
    iver.Kd*errDir;
rate = goalPitch - state.pitch(counter-1);

if rate < -iver.maxRate
    rate = -iver.maxRate;
elseif rate > iver.maxRate 
    rate = iver.maxRate;
end

dPitch = 0.1*rate;
state.pitch(counter) = state.pitch(counter-1) + dPitch + 0.2*randn(1);

if state.pitch(counter) < -iver.maxAngle
    state.pitch(counter) = -iver.maxAngle;
elseif state.pitch(counter) > iver.maxAngle
    state.pitch(counter) = iver.maxAngle;
end
state.pitch(counter) = state.pitch(counter) + 0.1*randn(1);
end