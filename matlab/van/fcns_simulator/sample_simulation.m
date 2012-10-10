clear all;

% execute mission setup script
mission_sample;

% sensor suite sample periods
samp_period = [1/10, 1/5, 1/3, 3]'; % xbow, rdi, paro, camera

% initial vehicle state in local-level reference frame
%     ^ X North
%     |
%     |
%     o----> Y East
%     Z Down
%
X_o = [  0;       % x
         0;       % y
        60;       % z
         0*DTOR;  % r
         0*DTOR;  % p
         0*DTOR;  % h
         0;       % vx
         0;       % vy
         0;       % vz
         0*DTOR;  % rr
         0*DTOR;  % pr
         0*DTOR;  % hr
      ];

% run trajector simulator
[X,t,mindx,dh] = runsim(goal_t,samp_period,X_o);

% show trajectory simulator results
plot_trajsim(X,t,mindx,dh,1);

% determine a bounding box for vehicle motion
bbox_t.x = [min(X(1,:)) max(X(1,:))];
bbox_t.y = [min(X(2,:)) max(X(2,:))];
bbox_t.z = [min(X(3,:)) max(X(3,:))];

% camera resolution and camera calibration matrix
res = [1280 1024];
K = [1700 0    640;
     0    1700 512;
     0    0    1 ];

% generate uniformly distributed XY scene points
fpi = 200;
alt = 3;
xlim = bbox_t.x + [-alt alt];
ylim = bbox_t.y + [-alt alt];
[X,Y,tag] = sim_scene_xy(fpi,xlim,ylim,alt,K,res,'grid');

% generate a smooth surface height
xcen = (xlim(2)+xlim(1))/2;
ycen = (ylim(2)+ylim(1))/2;
xamp = 1.5;
yamp = 0.5;
xlambda = 30;
ylambda = 15;
%Z = xamp*sin(2*pi*(X-xcen)/xlambda) + yamp*cos(2*pi*(Y-ycen)/ylambda);
Z = xamp*sin(2*pi*sqrt((X-xcen).^2+(Y-ycen).^2)/xlambda);

figure;
surf(reshape(X,sqrt(length(X)),[]), ...
     reshape(Y,sqrt(length(Y)),[]), ...
     reshape(Z,sqrt(length(Z)),[]));
