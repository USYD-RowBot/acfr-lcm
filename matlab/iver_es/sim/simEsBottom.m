function simEsBottom(counter)
global iver state
depth = state.depth(counter);
pitch = state.pitch(counter-1);
range = state.range(counter);
if range >= 0.3
    bottom = state.depth(counter) - state.alt(counter);
    esX = iver.dvl2es*cosd(pitch) - iver.radius*sind(pitch);
    esY = iver.dvl2es*sind(pitch) + iver.radius*cosd(pitch);
    rangeY = depth + esY + range*sind(pitch+3);
    rangeX = esX + range*min(cosd(pitch+3), cosd(pitch-3));
    run = rangeX;
    rise = rangeY - bottom;
    state.slopeEs(counter) = rise/run;
else
    state.slopeEs(counter) = 0;
end
end