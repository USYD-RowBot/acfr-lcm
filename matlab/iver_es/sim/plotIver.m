function plotIver(counter, ii, elapsed, dvlInd)
global map iver state horizon
xIver = state.x(counter);
pitch = state.pitch(counter-1);
depth = state.depth(counter);
alt = state.alt(counter);
altMod = state.altMod(counter);
range = state.range(counter);
% predRange = state.predRange(counter);
iS = max([1, ;floor(ii-((5/map.res)))]);
iSScale = ceil(iS/map.plotScale);
iE = min([floor(ii+(53/map.res)), size(map.x,2)]);
iEScale = min(ceil(iE/map.plotScale), size(map.xPlot,2));

% Creates bottom
track = map.xPlot(iSScale:iEScale); 
bottom = map.bottomPlot(iSScale:iEScale);

% % Creates DVL Planar bottom
% dvlXAft = xIver-dvlInd(2)*map.res*sind(iver.dvlAngle-pitch);
% dvlYAft = depth-dvlInd(2)*map.res*cosd(iver.dvlAngle-pitch);
% bottomDvlX = [map.x(iS) map.x(iE)];
% bottomDvlY = dvlYAft + ...
%     state.slopeDvl(counter)*[(map.x(iS)-dvlXAft) (map.x(iE)-dvlXAft)];
% 
% % Creates ES Planar bottom
% if state.slopeEs(counter) < 0
%     bottomEsX = xIver;
%     bottomEsY = depth - alt;
%     esMarkerType = 'y*';
% else
%     bottomEsX = [map.x(iS) map.x(iE)];
%     bottomEsY = dvlYAft + ...
%         state.slopeEs(counter)*[(map.x(iS)-xIver) (map.x(iE)-xIver)];
%     esMarkerType = 'y:';
% end

% Creates Horizon Planar bottom
if horizon.index == 0
    horizonXP = xIver;
    horizonYP = depth - alt;
    horPMarkerType = 'b*';
else
    horizonXP = [xIver (xIver + horizon.index + horizon.offset)];
    horizonYP = [(depth - alt) horizon.depth(horizon.index)];
    horPMarkerType = 'b-.';
end

% Creates Abort Planar bottom
if horizon.index == 0
    horizonXF = xIver;
    horizonYF = depth - 2;
    horFMarkerType = 'm*';
else
    horizonXF = [xIver (xIver + horizon.index + horizon.offset)];
    horizonYF = [(depth - 2) horizon.depth(horizon.index)];
    horFMarkerType = 'm--';
end

% Creates one full Iver
Iver(1,1) = xIver + iver.dvl2es*cosd(pitch);
Iver(1,2) = xIver + iver.dvl2es*cosd(pitch) - 2*iver.radius*sind(pitch);
Iver(1,3) = Iver(1,2) - iver.length*cosd(pitch);
Iver(1,4) = Iver(1,1) - iver.length*cosd(pitch);
Iver(1,5) = Iver(1,1);
Iver(2,1) = depth + iver.dvl2es*sind(pitch);
Iver(2,2) = depth + iver.dvl2es*sind(pitch) + 2*iver.radius*cosd(pitch);
Iver(2,3) = depth + (iver.dvl2es - iver.length)*sind(pitch) + ...
    2*iver.radius*cosd(pitch);
Iver(2,4) = depth + (iver.dvl2es - iver.length)*sind(pitch);
Iver(2,5) = Iver(2,1);

% Creates DVL lines
dvl(1,:) = [xIver - dvlInd(2)*map.res*sind(iver.dvlAngle - pitch) ...
    xIver  xIver + dvlInd(1)*map.res*sind(iver.dvlAngle + pitch)];
dvl(2,:) = [depth - dvlInd(2)*map.res*cosd(iver.dvlAngle - pitch) ...
    depth  depth - dvlInd(1)*map.res*cosd(iver.dvlAngle + pitch)];

%Creates Range Cone
xEs = xIver + iver.dvl2es*cosd(pitch) - iver.radius*sind(pitch);
yEs = depth + iver.dvl2es*sind(pitch) + iver.radius*cosd(pitch);
esAngle = (-3:1:3) + pitch;
esTemp(1,:) =  xEs + range*cosd(esAngle);
esTemp(2,:) =  yEs + range*sind(esAngle);
es(1,:) = [xEs esTemp(1,:) xEs];
es(2,:) = [yEs esTemp(2,:) yEs];

% %Creates Predicted Range Cone
% esPredTemp(1,:) =  xEs + predRange*cosd(esAngle);
% esPredTemp(2,:) =  yEs + predRange*sind(esAngle);
% esPred(1,:) = [xEs esPredTemp(1,:) xEs];
% esPred(2,:) = [yEs esPredTemp(2,:) yEs];

%Creates Horizon Scatter data
horizonScatterX = (xEs + horizon.offset) + (0:49);

% Plots Everything
figure(1)
set(1,'Units','pixels','Position', [100 400 1000 500]);
plot(track, bottom, 'k');
hold on
% plot(bottomDvlX, bottomDvlY, 'Color', [1 0.5 0], 'LineStyle', ':');
% plot(bottomEsX, bottomEsY, esMarkerType);
plot(horizonXP, horizonYP, horPMarkerType);
plot(horizonXF, horizonYF, horFMarkerType);
plot(Iver(1,:), Iver(2,:), 'y');
plot(dvl(1,:), dvl(2,:), 'c');
plot(es(1,:), es(2,:), 'g');
% plot(esPred(1,:), esPred(2,:), 'r-.');
colormap([1 0 0; 1 1 0; 0 1 0]);
caxis([-1 1])
scatter(horizonScatterX, horizon.depth, 9, horizon.good)
axis equal; grid off;
if horizon.index > 0;
    hDepth = num2str(horizon.depth(horizon.index));
else
    hDepth = ' ';
end
title({['Goal Alititude = ',num2str(iver.goalAlt), ...
    'm || Actual Alititude = ', num2str(alt), ...
    'm || Modulated Alititude = ', num2str(altMod), 'm || Range = ', ...
    num2str(range), 'm || Elapsed Time = ', num2str(elapsed), 'sec'];...
    ['Horizon Distance = ',num2str(horizon.offset + horizon.index), ...
    'm || Horizon Depth = ', hDepth, 'm || Horizon Angle = ', ...
    num2str(atan(horizon.slope)*180/pi), 'deg || Flag Angle = ', ...
    num2str(atan(horizon.flagSlope)*180/pi), ' deg']})
xlabel('Distance along Track [m]')
ylabel('Depth [m]')
xlim([track(1) track(end)]);
ylim([depth-10 depth+10])
drawnow
hold off
end