function data = sim_es(botType, cont, dive)
close all
if ~exist('cont', 'var'), cont = 'on'; end
if ~exist('dive', 'var'), dive = 'off'; end

global iver map state horizon control

map.res = 0.01;
map.botType = botType;
map.dive = dive;
map.x = [];
map.bottom = [];
map.gridRes = 0.5;
map.xPlot = [];
map.bottomPlot = [];
map.plotScale = 10;

iver.radius = 0.075; % m
iver.length = 1.9;
iver.noseLength = 0.41; % m
iver.bodyLength = 1.22; % m
iver.tailLength = 0.47; % m
iver.dvl2es = 0.3; % m
iver.dvlAngle = 30*cosd(45); % deg
iver.esAngle = 6; % deg
iver.esRangeMax = 50; % m
iver.stabAngle = 2.5; % deg
iver.maxRate = 10; % deg/s
iver.maxAngle = 25; % deg
iver.maxSlope = tand(iver.maxAngle);
iver.turnSlope = tand(iver.maxAngle-5);
iver.greenSlope = tand(iver.maxAngle-10);
iver.yellowSlope = iver.greenSlope/2;
iver.goalAlt = 3; % m
iver.warnAlt = 2; % m
iver.Kp = 50; 
iver.Ki = 0.1; 
iver.Kd = 1;
iver.errProp = 0; 
iver.errInt = 0;
iver.esFreq = 1/3;

state.x = zeros(1,3e4);
state.depth = zeros(1,3e4);
state.speed = ones(1,3e4); % m/s
state.alt = zeros(1,3e4);
state.heading = zeros(1,3e4);
state.altMod = zeros(1,3e4);
state.altDiff = zeros(1,3e4);
state.pitch = zeros(1,3e4);
state.dPitch = zeros(1,3e4);
state.goalPitch = zeros(1,3e4);
state.range = zeros(1,3e4);
state.predRange = zeros(1,3e4);
state.slopeDvl = zeros(1,3e4);
state.slopeEs = zeros(1,3e4);
state.slopeControl = zeros(1,3e4);

horizon.offset = 0.5;
horizon.depth = zeros(1,50);
horizon.good = -1*ones(1,50);
horizon.slope = 0;
horizon.index = 0;
horizon.flagSlope = 0;

control.cont = cont;
control.abort = 0;
control.loopAround = 0;
control.dangerCoords = [0 0];
control.altModConst = 0;
control.altModFloat = 0;

[n start] = createBottom();

finish = 55;
iStart = floor(start/map.res); iFinish = floor(n-(finish/map.res));
elapsed = 0;
counter = 4;
state.x(3) = map.x(iStart-ceil(0.1/map.res));
state.alt(3) = state.depth(3) + 100;
state.altMod(3) = state.alt(3);
state.altDiff(3)  = state.alt(3) - iver.goalAlt;
% rise = [0 0 0]; run = rise;
counterES = 3;
dx = ceil((0.1/map.res)*cosd(state.pitch(1))*cosd(state.heading(1)));
clear botType cont dive finish start n
tic
%% Runs Simulation 
for i = iStart:dx:iFinish
    state.heading(counter) = state.heading(counter-1);
    state.x(counter) = map.x(i);
    state.depth(counter) = state.depth(counter-1) + ...
        dx*map.res*sind(state.pitch(counter-1)+iver.stabAngle);
    
    dvlInd = findAlt(counter, i);
%     rri = mod(counter-1,3) + 1;
%     [rise(rri) run(rri)]  = simDvlBottom(counter, dvlInd);
        
    if mod(counter, floor(1/iver.esFreq))
%         predictRange(counter, rise, run);
        findRange(counter, i);
%         simEsBottom(counter);
        dist = state.x(counter) - state.x(counterES);
        updateHorizon(counter, dist);
        if strcmp(control.cont, 'on')
            altModulation(counter);
        end
        counterES = counter;
    else
        state.predRange(counter) = state.predRange(counterES);
        state.range(counter) = state.range(counterES);
        state.slopeEs(counter) = state.slopeEs(counterES);
    end
    state.altMod(counter) = state.alt(counter) - ...
        control.altModConst - control.altModFloat;
    getpitch(counter);
    if ~mod(i,3)
        plotIver(counter, i, elapsed, dvlInd);
    end
    elapsed = elapsed + 0.1;
    dx = ceil((0.1/map.res)*cosd(state.pitch(counter))*...
        cosd(state.heading(counter)));
    if state.alt(counter) <= 0 || ...
            control.abort == 1 || control.loopAround ==1
        break
    end
    counter = counter + 1;
end
%% Cleans stuff up and plots
realTime=toc;
disp(['Sim Rate = ', num2str(elapsed/realTime)])
data = parseData(iStart, i);
plotUseful(data)
end


%%                          SUB-Functions


%% Modulates Altitude for control
function altModulation(counter)
global iver state horizon control
control.altModFloat = 0;
if horizon.flagSlope > iver.maxSlope && horizon.index < 8;
    control.abort = 1;
    disp('ABORT!')
elseif horizon.flagSlope > iver.turnSlope && horizon.index < 15;
    control.loopAround = 1;
%     control.altModConst = control.altModConst + 2;
    disp('Turn Around');
elseif horizon.index == 0
    return
elseif horizon.good(horizon.index) == 1 &&...
        horizon.slope > iver.greenSlope
    dist = horizon.index + horizon.offset;
    pseudoBottomDepth = horizon.depth(horizon.index) - ...
        iver.greenSlope*dist;
    pseudoAlt = state.depth(counter) - pseudoBottomDepth;
    control.altModFloat = state.alt(counter) - pseudoAlt;
elseif horizon.good(horizon.index) < 1 &&...
        horizon.slope > iver.yellowSlope
    dist = horizon.index + horizon.offset;
    pseudoBottomDepth = horizon.depth(horizon.index) - ...
        iver.yellowSlope*dist;
    pseudoAlt = state.depth(counter) - pseudoBottomDepth;
    control.altModFloat = state.alt(counter) - pseudoAlt;
end
end

%% Creates Bottom Profile
function [n start] = createBottom()
global map state iver
switch map.botType
    case 'flat'
        slope = zeros(1,(500/map.res));

    case 'sine'
        slope = [zeros(1,(50/map.res)) sin((2*pi/(100/map.res))*(1:(200/map.res))) ...
            5*sin((2*pi/(100/map.res))*(1:(200/map.res))) ...
            10*sin((2*pi/(100/map.res))*(1:(200/map.res))) ...
            20*sin((2*pi/(100/map.res))*(1:(100/map.res))) ...
            zeros(1,(100/map.res))];
        
    case 'rand'
        slope = [zeros(1,(50/map.res)) .3*randn(1,(850/map.res)) ...
            zeros(1,(50/map.res))];
       
    case 'ramp'
        slope = [zeros(1,(50/map.res)) linspace(0,1,(10/map.res)) ...
            ones(1,(30/map.res)) linspace(1,0,(10/map.res)) ...
            zeros(1,(30/map.res)) linspace(0,5,(10/map.res)) ...
            5*ones(1,(30/map.res)) linspace(5,0,(10/map.res)) ...
            zeros(1,(30/map.res)) linspace(0,10,(10/map.res)) ...
            10*ones(1,(30/map.res)) linspace(10,0,(10/map.res)) ...
            zeros(1,(30/map.res)) linspace(0,20,(10/map.res)) ...
            20*ones(1,(30/map.res)) linspace(20,0,(10/map.res)) ...
            zeros(1,(100/map.res))];
       
    case 'step'
        slope = [zeros(1,(50/map.res)) 1*ones(1,(30/map.res)) ...
            zeros(1,(50/map.res)) 2.5*ones(1,(30/map.res)) ...
            zeros(1,(50/map.res)) 5*ones(1,(30/map.res)) ...
            zeros(1,(50/map.res)) 10*ones(1,(30/map.res)) ...
            zeros(1,(50/map.res)) 20*ones(1,(30/map.res)) zeros(1,(100/map.res))];
       
    case 'ship'
        map.deck = input('Enter ship deck height above bottom (m): ');
        slope = [zeros(1,(100/map.res)) map.deck*ones(1,(10/map.res))];
        for numPass = 1:1
            slope = [slope zeros(1,(70/map.res)) ...
                map.deck*ones(1,(10/map.res))];
        end
        slope = [slope zeros(1,(70/map.res)) ...
            map.deck*ones(1,(10/map.res)) zeros(1,(100/map.res))];
end
n = size(slope,2);
if strcmp(map.dive,'on')
    start = 150;
    slope = [zeros(1,350/map.res) slope];
    n = size(slope,2);  
    state.depth(3) = -1;
else
    start = 45;
    state.depth(3) = -100 + iver.goalAlt;
end
texture = addTexture(n);
roughness = 0.025*randn(1,n);
map.bottom = -100*ones(1,n) + slope + texture + roughness;
map.x = (map.res:map.res:n*map.res);
for j = 1:map.plotScale:n;
    map.xPlot(end+1) = map.x(j);
    map.bottomPlot(end+1) = map.bottom(j);
end
end
% Creates bottom texture (i.e. rocks and vallies) 
function texture = addTexture(n)
global map
i = 1;
tempTexture = zeros(1,n);
while i < n;
    type = ceil(12*rand(1));
    len = ceil(floor(0.25/map.res)+(floor(5/map.res)*rand(1)));
    if type == 1 || type == 2 || type == 3% No Rock
        rock = zeros(1,len);
    elseif type == 4 || type ==5 % Ramp Rock
        height = 0.3*randn(1);
        frontRamp = ceil((len-2)*rand(1));
        backRamp = ceil((len-frontRamp)*rand(1));
        top = len - frontRamp - backRamp;
        rock = [linspace(0,height,frontRamp) height*ones(1,top) ...
            linspace(height,0,backRamp)];
    elseif type == 6 || type == 7 % Ramp Rock
        height = 0.3*randn(1);
        backRamp = ceil((len-2)*rand(1));
        frontRamp = ceil((len-backRamp)*rand(1));
        top = len - frontRamp - backRamp;
        rock = [linspace(0,height,frontRamp) height*ones(1,top) ...
            linspace(height,0,backRamp)];
    elseif type == 8 % Step Rock
        height = 0.3*randn(1);
        rock = height*ones(1,len);
    elseif type == 9 || type == 10 % Round Rock
        height = 0.3*randn(1);
        dummy = 1:len;
        rock = height*(sin((pi/len)*dummy));
    elseif type == 11 || type == 12 % Round Rock
        height = 0.3*randn(1);
        dummy = 1:len;
        rock = height*(sin((pi/len)*dummy).^2);
    end
    tempTexture(i:i+len-1) = rock;
    i = i + len;
end
texture = tempTexture(1:n);
end

%% Finds DVL measured Altitude
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
% Ray Traces Fwd DVL Beam
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
% Ray Traces Aft DVL Beam
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

%% Finds ES measured Range
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
% Ray traces ES Beam
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

%% Determines new Pitch
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

%% Parse useful data into struct
function data = parseData(iStart, i)
global map iver state
data.map.res = map.res;
data.map.botType = map.botType;
data.map.dive = map.dive;
data.map.x = map.x(iStart:i);
data.map.bottom = map.bottom(iStart:i);
data.map.gridRes = map.gridRes;
data.map.xPlot = map.x(floor(iStart/map.plotScale):ceil(i/map.plotScale));
data.map.bottomPlot = map.bottom(floor(iStart/map.plotScale): ...
    ceil(i/map.plotScale));
data.map.plotScale = map.plotScale;

data.iver = iver;

data.state.x = state.x(1:counter);
data.state.depth = state.depth(1:counter);
data.state.speed = state.speed(1:counter);
data.state.alt = state.alt(1:counter);
data.state.heading = state.heading(1:counter);
data.state.altMod = state.altMod(1:counter);
data.state.altDiff = state.altDiff(1:counter);
data.state.pitch = state.pitch(1:counter);
data.state.dPitch = state.dPitch(1:counter);
data.state.goalPitch = state.goalPitch(1:counter);
data.state.range = state.range(1:counter);
data.state.predRange = state.predRange(1:counter);
data.state.slopeDvl = state.slopeDvl(1:counter);
data.state.slopeEs = state.slopeEs(1:counter);
data.state.slopeControl = state.slopeControl(1:counter);
end

%% Plots Simulation
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

%% Plots of applicabale Data After Simulation
function plotUseful(data)
figure(2), clf(2)
[AX2, dummy1, dummy2] = plotyy(data.map.xPlot, data.map.bottomPlot, ...
    data.state.x, data.state.alt);
legend([dummy1 dummy2], 'Bottom', 'Altitude')
set(get(AX2(1),'Ylabel'),'String','Bottom Depth [m]') 
set(dummy1, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 1, ...
    'Color', [0.5 0.5 0.5])
set(get(AX2(2),'Ylabel'),'String','Altitude [m]')
set(dummy2, 'LineStyle', ':', 'Marker', '.', 'MarkerSize', 1, ...
    'Color', 'r')
xlabel('Distance along trackline [m]')

% figure(3), clf(3)
% dummy34 = plot(data.state.x, data.state.goalPitch, ...
%     data.state.x, data.state.pitch);
% set(dummy34(1), 'LineStyle', ':', 'Marker', '.', 'MarkerSize', 1, ...
%     'Color', [0.5 0.5 0.5])
% set(dummy34(2), 'LineStyle', ':', 'Marker', '.', 'MarkerSize', 1, ...
%     'Color', 'r');
% legend('Goal Pitch', 'Pitch')
% ylabel('pitch [deg]');
% xlabel('Distance along trackline [m]')
% ylim([-35 35])
% 
% figure(4), clf(4)
% [AX4, dummy5, dummy6] = plotyy(data.map.xPlot, data.map.bottomPlot, ...
%     data.state.x, data.state.range);
% legend([dummy5 dummy6], 'Bottom', 'Range')
% set(get(AX4(1),'Ylabel'),'String','Bottom Depth [m]')
% set(dummy5, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 1, ...
%     'Color', [0.5 0.5 0.5])
% set(get(AX4(2),'Ylabel'),'String','Range Returns [m]') 
% set(dummy6, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 2, ...
%     'Color', 'r')
% xlabel('Distance along trackline [m]')
% 
% figure(5), clf(5)
% dummy78 = plot(data.state.x, data.state.predRange, ...
%     data.state.x, data.state.range);
% set(dummy78(1), 'LineStyle', 'none', 'Marker', 'x', 'MarkerSize', 2, ...
%     'Color', [0.5 0.5 0.5])
% set(dummy78(2), 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 2, ...
%     'Color', 'r');
% legend('Predicted Range', 'Range')
% ylabel('Range [m]');
% xlabel('Distance along trackline [m]')

figure(6), clf(6)
dummy910 = plot(data.state.x, data.state.altMod, ...
    data.state.x, data.state.alt);
set(dummy910(1), 'LineStyle', 'none', 'Marker', 'x', 'MarkerSize', 4, ...
    'Color', [0.5 0.5 0.5])
set(dummy910(2), 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 2, ...
    'Color', 'r');
legend('Control Altitude', 'Actual Altitude')
ylabel('Altitude [m]');
xlabel('Distance along trackline [m]')
end

%% Predicts Range given a DVL Planar Bottom
function predictRange(counter, rise, run)
global iver state
pitch = state.pitch(counter-1);
alt = state.alt(counter);
slopeDvl = state.slopeDvl(counter);
beamAngle = pitch - 3;
bottomAngle = mean(atan2(rise, -run))*180/pi;
if bottomAngle < beamAngle
    state.predRange(counter) = 0;
else
    phi = -(beamAngle + bottomAngle);
    if phi < 0;
        state.predRange(counter) = 0;
        return
    end
    esX = iver.dvl2es*cosd(pitch) - iver.radius*sind(pitch);
    esY = iver.dvl2es*sind(pitch) + iver.radius*cosd(pitch);
    altES = alt + esY + slopeDvl*esX;
    state.predRange(counter) = (altES*sind(90-bottomAngle))/sind(phi);
    if state.predRange(counter) < 0.30
        state.predRange(counter) = 0.30;
    elseif state.predRange(counter) > 50
        state.predRange(counter) = 0;
    end
end
end

%% Creates DVL Planar Bottom
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

%% Creates ES Planar Bottom
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

%% Updated Horizon Data
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