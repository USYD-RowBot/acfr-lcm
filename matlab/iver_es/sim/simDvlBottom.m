function [rise run] = simDvlBottom(counter, dvlInd)
global map iver state
pitch = state.pitch(counter-1);
rise = dvlInd(1)*map.res*cosd(iver.dvlAngle+pitch) - ...
    dvlInd(2)*map.res*cosd(iver.dvlAngle-pitch);
run = - dvlInd(1)*map.res*sind(iver.dvlAngle+pitch) - ...
    dvlInd(2)*map.res*sind(iver.dvlAngle-pitch);
slope = rise/run;
state.slopeDvl(counter) = slope;
end