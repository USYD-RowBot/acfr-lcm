function findRange(counter, i)
global map iver state
jMax = 50/map.res;
j = 0;
angMid = state.pitch(counter-1);
ii = floor(i + (iver.dvl2es/map.res)*cosd(angMid));
angDown = -3 + angMid; angUp = 3 + angMid;
depthES = (state.depth(counter) + iver.radius*cosd(angMid));
j = rayTraceES(ii, j, jMax, angDown, angMid, angUp, map.res, ...
    depthES, map.bottom, 1);
j = rayTraceES(ii, j, jMax, angDown, angMid, angUp, map.res, ...
    depthES, map.bottom, 0.1);
j = rayTraceES(ii, j, jMax, angDown, angMid, angUp, map.res, ...
    depthES, map.bottom, 0.01);
j= j+1;
state.range(counter) = j*map.res;
if state.range(counter) < 0.3;
    state.range(counter) = 0.3;
elseif state.range(counter) >= 50-map.res
    state.range(counter) = 0;
end
end

function j = rayTraceES(ii, j, jMax, angDown, angMid, angUp, res, ...
    depthES, bottomY, step)
jDown = floor(ii+j*cosd(angDown));
jMid = floor(ii+j*cosd(angMid));
jUp = floor(ii+j*cosd(angUp));
checkRangeDown = (depthES + j*res*sind(angDown)) - bottomY(jDown);
checkRangeMid = (depthES + j*res*sind(angMid)) - bottomY(jMid);
checkRangeUp = (depthES + j*res*sind(angUp)) - bottomY(jUp);
while (checkRangeDown > 0 && checkRangeMid > 0 && checkRangeUp > 0 && ...
        j  < jMax)
    j = min(j+floor(step/res), jMax);
    jDown = floor(ii+j*cosd(angDown));
    jMid = floor(ii+j*cosd(angMid));
    jUp = floor(ii+j*cosd(angUp));
    checkRangeDown = (depthES + j*res*sind(angDown)) - bottomY(jDown);
    checkRangeMid = (depthES + j*res*sind(angMid)) - bottomY(jMid);
    checkRangeUp = (depthES + j*res*sind(angUp)) - bottomY(jUp);
end
j = max(j - floor(2*step/res), 0);
end