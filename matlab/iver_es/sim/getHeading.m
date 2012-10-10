function getHeading(counter)
global iver state
lastHeading = state.heading(counter-1);
if state.waypoint(counter) == 0;
    state.goalHeading(counter) = 180;
else
    state.goalHeading(counter) = 0;
end

if lastHeading ~= state.goalHeading(counter); 
    rate = iver.maxRate;
else
    state.heading(counter) = state.heading(counter-1);
    return
end
dHeading = 0.1*rate;
state.heading(counter) = state.heading(counter-1) + dHeading;
if state.heading(counter) < 0
    state.heading(counter) = state.heading(counter) + 360;
elseif state.heading(counter) >= 360
    state.heading(counter) = state.heading(counter) - 360;
end
if abs(state.goalHeading(counter)-state.heading(counter)) < 1.5
    state.heading(counter) = state.goalHeading(counter);
end
state.pitch(counter) = state.pitch(counter) + 0.1*randn(1);
end