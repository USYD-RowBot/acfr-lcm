function updateHorizon(counter, dist)
global iver state horizon
pitch = state.pitch(counter-1);
if state.range(counter) == 0
    state.range(counter) = 50;
end
horizon.offset = horizon.offset - dist;
while horizon.offset < 0
    horizon.offset = horizon.offset + 1;
    for i = 1:49
        horizon.depth(i) = horizon.depth(i+1);
        horizon.good(i) = horizon.good(i+1);
    end
    horizon.depth(50) = 0;
    horizon.good(50) = -1;
end
esDepth = state.depth(counter) + ...
    iver.dvl2es*sind(pitch) + ...
    iver.radius*cosd(pitch);
esX = iver.dvl2es*cosd(pitch) - ...
    iver.radius*sind(pitch);
ii = 0;
iiRange = ii + horizon.offset;
while iiRange < (state.range(counter) - 1)*cosd(pitch)
    horizon.depth(ii+1) = min(esDepth + ...
        iiRange*sind(pitch-3), ...
        horizon.depth(ii+1));
    if  horizon.good(ii+1) == 0
        horizon.good(ii+1) = 1;
    end
    ii = ii + 1;
    iiRange = ii + horizon.offset;
end
tempDepthMax = esDepth + iiRange*sind(pitch+3);
tempDepthMin = esDepth + iiRange*sind(pitch-3);
if tempDepthMin > horizon.depth(ii+1) && horizon.good(ii+1) == 1;
    horizon.depth(ii+1) = tempDepthMax;
    horizon.good(ii+1) = 0;
elseif tempDepthMax > horizon.depth(ii+1) && horizon.good(ii+1) == 0;
    horizon.depth(ii+1) = tempDepthMax;
elseif horizon.good(ii+1) == -1 && state.range(counter) ~= 50;
    horizon.depth(ii+1) = tempDepthMax;
    horizon.good(ii+1) = 0;
elseif horizon.good(ii+1) == -1
    horizon.depth(ii+1) = tempDepthMax;
end
bottomDepth = state.depth(counter) - state.alt(counter);
horizon.slope = 0;
horizon.index = 0;
for jj = 2:50
    if horizon.good(jj) > -1
        rise = horizon.depth(jj) - bottomDepth;
        run = esX + jj + horizon.offset;
        tempSlope = rise/run;
        if tempSlope > horizon.slope
            horizon.slope = tempSlope;
            horizon.index = jj;
        end
    end
end
if horizon.index > 0
    rise = horizon.depth(horizon.index) - (state.depth(counter) - 2);
    run = esX + horizon.index + horizon.offset;
    horizon.flagSlope = rise/run;
else
    horizon.flagSlope = 0;
end
if state.range(counter) == 50
    state.range(counter) = 0;
end
end