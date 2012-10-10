
clear

%A script that sets up a simulator for structure and motion algorithms

%camera info-----------------------------------------------
config.data.dpath = '/files1/data/bermuda02/';
config.data.cfile = 'Calib_Results_20021125.m';
% load camera calibration results
[K,kr,kt] = load_camera_calib(config.data.dpath,config.data.cfile);
resx = 1280;
resy = 1024;
res = [resx;resy];
sigma = 1;

% assume roughly level vehicle

ncam = 9; % number cameras (viewS)

ti = [0;0;0]; % start position
tf = [0;5;0]; % end position
rphi = [0;0;0]; % start orientation
rphf = [0;0;1]; % end orientation
pose_i = [rphi;ti];
pose_f = [rphf;tf];
stdvec = [0.01;0.01;0.02;0.2;0.2;0.1];
stdmotion = [0.02;0.02;0.2;0.4;0.4;0.2];

posemat = generate_pose(pose_i,pose_f,ncam,stdmotion);

alt = 3.5;
zref = -10; % reference plane
min_lim = min([ti tf],2)-4; %extend limits to account for field of view
max_lim = max([ti tf],2)+4;
xlim = [min_lim(1) max_lim(1)];
ylim = [min_lim(2) max_lim(2)];

fpimage = 500; % number of 3D feature points per image

% Generate scene
% add an option for type of scene 'general','planar','mixed'
% add an option for 'noise level' on z coord

%[Xmat_true,tag,fpm2] = generate_scene(fpimage,xlim,ylim,alt,K,res);
[X,Y,tag] = generate_XY(fpimage,xlim,ylim,alt,K,res);

% calculate Z however you want (account for altitude)
% Z = randn(length(X),1)-alt;
Z = 0.25*sin(X+Y)+0.25*cos(0.5*X-0.5*Y)-alt;
% assemble Xmat
Xmat_true = [X Y Z];

% add reference depth
posemat(6,:) = posemat(6,:) + zref;
Xmat_true(:,3) = Xmat_true(:,3) + zref;

%given camera poses, generate images 
[im_struc,im_struc_true, tag_struc] = generate_images(Xmat_true,tag,posemat,K,res,sigma);


%generate correspondence matrices: matchmat
[matchmat_true, clinks_true] = generate_xpondences(im_struc);

