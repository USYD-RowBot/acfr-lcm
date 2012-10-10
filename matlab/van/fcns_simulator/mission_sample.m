mode_type_t = mode_type_struct;

% set depth, roll, pitch variation for entire survey
% note, these can be changed on a behavior by behavior basis
param_t.ascent_rate   = 0.2;  % m/s
param_t.ascent_period = 60; % seconds
param_t.pitch_rate    = 1*DTOR;
param_t.pitch_period  = 30; % seconds
param_t.roll_rate     = 1*DTOR;
param_t.roll_period   = 30; % seconds

ii = 1;
% do 360 degrees of a circle of radius 15 m
goal_t(ii).XYMode = mode_type_t.XY_CIRCLE;
goal_t(ii).param = param_t;
goal_t(ii).param.angle = 360*DTOR;
goal_t(ii).param.radius = 15;
goal_t(ii).param.speed = 0.25;

% do a survey grid of 5 N/S lines 50m long and 1.5m apart
% starting at [0,0]
tmp = calc_grid([0,0],0*DTOR,50,1.5,5,0.35,param_t);
goal_t = [goal_t,tmp];

ii=length(goal_t)+1;
% drive to a waypoint (30,-20) @ 0.75m/s
goal_t(ii).XYMode = mode_type_t.XY_GOTO;
goal_t(ii).param = param_t;
goal_t(ii).param.stop  = [30,-20];
goal_t(ii).param.speed = 0.75;

ii=length(goal_t)+1;
% drive a transecting line across grid
goal_t(ii).XYMode = mode_type_t.XY_LINE;
goal_t(ii).param = param_t;
goal_t(ii).param.start = [25,-20];
goal_t(ii).param.stop  = [25,10];
goal_t(ii).param.speed = 0.35;

ii=length(goal_t)+1;
% drive a heading of 90 for 25m while doing a "squiggle"
goal_t(ii).XYMode = mode_type_t.XY_SQUIGGLE;
goal_t(ii).param = param_t;
goal_t(ii).param.heading = 90*DTOR;
goal_t(ii).param.heading_rate = 20*DTOR;
goal_t(ii).param.heading_period = 15; % seconds
goal_t(ii).param.distance = 25; % [m]
goal_t(ii).param.speed = 0.35;
