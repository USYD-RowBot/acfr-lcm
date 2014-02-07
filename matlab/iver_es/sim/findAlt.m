function dvlInd = findAlt(counter, ii)
global map iver state
dvlInd = [0 0];
pitch = state.pitch(counter-1);
depth = state.depth(counter);
alt = state.alt(counter);
%check fwd beam
dvlInd(1) = rayTraceFwd(ii, dvlInd(1), iver.dvlAngle, pitch,...
    depth, map.res, map.bottom, 1);
dvlInd(1) = rayTraceFwd(ii, dvlInd(1), iver.dvlAngle, pitch,...
    depth, map.res, map.bottom, 0.1);
dvlInd(1) = rayTraceFwd(ii, dvlInd(1), iver.dvlAngle, pitch,...
    depth, map.res, map.bottom, 0.01);

% check aft beam
dvlInd(2) = rayTraceAft(ii, dvlInd(2), iver.dvlAngle, pitch,...
    depth, map.res, map.bottom, 1);
dvlInd(2) = rayTraceAft(ii, dvlInd(2), iver.dvlAngle, pitch,...
    depth, map.res, map.bottom, 0.1);
dvlInd(2) = rayTraceAft(ii, dvlInd(2), iver.dvlAngle, pitch,...
    depth, map.res, map.bottom, 0.01);

% take min alt
dvlInd = floor(dvlInd + 0.01/map.res);
state.alt(counter) = min(dvlInd(1)*map.res*cosd(iver.dvlAngle+pitch), ...
    dvlInd(2)*map.res*cosd(iver.dvlAngle-pitch));
state.altDiff(counter) = alt - iver.goalAlt;
end

%% %%%%%%%%%%%%%%%%%%% Ray Traces Fwd DVL Beam %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dvlInd = rayTraceFwd(ii, dvlInd, dvlAngle, pitch, depth, res, ...
    bottom, step)
j = floor(ii + (dvlInd*sind(dvlAngle + pitch)));
checkAltFwd = depth - dvlInd*res*cosd(dvlAngle + pitch);
while (checkAltFwd > bottom(j) &&  dvlInd < 150/res)
    dvlInd = floor(dvlInd + step/res);
    j = floor(ii+(dvlInd*sind(dvlAngle+pitch)));
    checkAltFwd = depth - dvlInd*res*cosd(dvlAngle + pitch);
end
dvlInd = max(floor(dvlInd- 2*step/res),0);
end

%% %%%%%%%%%%%%%%%%%%% Ray Traces Aft DVL Beam %%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dvlInd = rayTraceAft(ii, dvlInd, dvlAngle, pitch, depth, res, ...
    bottom, step)
j = floor(ii + (dvlInd*sind(dvlAngle - pitch)));
checkAltFwd = depth - dvlInd*res*cosd(dvlAngle-pitch);
while (checkAltFwd > bottom(j) &&  dvlInd < 150/res)
    dvlInd = floor(dvlInd + step/res);
    j = floor(ii - (dvlInd*sind(dvlAngle+pitch)));
    checkAltFwd = depth - dvlInd*res*cosd(dvlAngle-pitch);
end
dvlInd = max(floor(dvlInd- 2*step/res),0);
end
