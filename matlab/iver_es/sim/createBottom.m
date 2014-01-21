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

%% %%%%%%%%%%%%%%%% Creates bottom texture (i.e. rocks) %%%%%%%%%%%%%%%%%%%
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