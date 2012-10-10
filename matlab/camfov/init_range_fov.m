function user = init_range_fov ()

%  Need user defined range for each value
%  Design parameters:
%  1) offset: offset from the target wall (hull) to camera [meter]
%  2) overlap: overlap ratio between two images            [%] (0-100)
%  3) fps: frame per second                                [ ]
%  4) vel: velocity of the vehicle                         [m/s]
%
%  Intialize global variables {min, tick, max, val}
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2010-08-20      ak          Created from camfov

% offset [meter]
user.offset_min = 0.5;
user.offset_tick = 0.5;
user.offset_max = 4;
user.offset = 0.5;
user.offset_range = user.offset_min:user.offset_tick:user.offset_max;

% fps [number of frames per second]
user.fps_min = 0.25;
user.fps_tick = 0.25;
user.fps_max = 4;
user.fps = 1;
user.fps_range = user.fps_min:user.fps_tick:user.fps_max;

% overlap [%]
user.overlap_min = 10;
user.overlap_tick = 10;
user.overlap_max = 100;
user.overlap = 50;
user.overlap_range = user.overlap_min:user.overlap_tick:user.overlap_max;

% velocity [m/s]
user.vel_min = 0.1;
user.vel_tick = 0.1;
user.vel_max = 1;
user.vel = 0.5;
user.vel_range = user.vel_min:user.vel_tick:user.vel_max;

% cross track line space
user.xspace_min = 0.1;
user.xspace_tick = 0.1;
user.xspace_max = 1;
user.xspace = 0.5;
user.xspace_range = user.xspace_min:user.xspace_tick:user.xspace_max;

% fov
user.FOV_along = 41.1086*DTOR; 
user.FOV_cross = 33.5269*DTOR;
user.FOV = user.FOV_along;

% initial plot = fps
user.slider_min = user.fps_min;
user.slider_max = user.fps_max;
user.slider_val = user.fps;

%  Define color code for the graph and slide bar range
user.color_set = ['y';'r';'g';'b';'k';'m';'c';];

