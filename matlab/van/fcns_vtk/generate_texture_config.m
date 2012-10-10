function ncam = generate_texture_config(pose,K,res,imgnum,texdir,filename)
% generates camera poses for vtk for texture projection from matlab
% poses and image configuration
% DATE      WHO     WHAT
%---------  ---     ----------------------------------------
% 20040302  OP      Commented
% 20050510  rme     got rid of OP data structure specific arguments


invK = inv(K);

% Set the aspect ratio of a perpendicular cross-section of the
% the projector's frustum.  The aspect ratio consists of three 
% numbers:  (x, y, z), where x is the width of the 
% frustum, y is the height, and z is the perpendicular
% distance from the focus of the projector.

% determine the symmetric part of the image
minpoints = invK*[0 0 1]'
maxpoints = invK*[res(1)-1 res(2)-1 1]'

symdelta = min(abs(maxpoints),abs(minpoints))

% the aspect ratio
aratio = [2*symdelta(1:2);1];

% images (to be used as textures) must be cropped accordingly


%# 20040127 projecting texture
%vtkProjectedTexture cam10
%    cam10 SetPosition  -0.2204 4.3039 16.6165

ncam = size(pose,2);

%filename = 'texture_config.dat';
fid = fopen(filename,'w');
% print texture directory
fprintf(fid,'%s\n',texdir);

for k = [1:ncam];

  [fc_w,up_w] = pose2fcpoint(pose(:,k));
  %planevec = pose2planes(pose(:,k),symdelta);
  [points,normals] = pose2planes(pose(:,k),symdelta);
  
  % create points and normals to describe planes for clipping
  for i = 1:6
    fprintf(fid,'%.3f %.3f %.3f\n',points(1,i),points(2,i),points(3,i));
  end 
  % normals
  for i = 1:6
    fprintf(fid,'%.3f %.3f %.3f\n',normals(1,i),normals(2,i),normals(3,i));
  end
  % texture image filename
  wildcard = sprintf('%s/*.%04d.png',texdir,imgnum(k));
  info = dir(wildcard);
  texture_filename = info.name;
  fprintf(fid,'%s\n',texture_filename);
  
  % projected texture postion
  fprintf(fid,'%.3f %.3f %.3f\n',pose(4,k),pose(5,k),pose(6,k));
  % projected texture focalpoint
  fprintf(fid,'%.3f %.3f %.3f\n',fc_w(1),fc_w(2),fc_w(3));
  % projected texture up vector
  fprintf(fid,'%.3f %.3f %.3f\n',up_w(1),up_w(2),up_w(3));
  % projected texture aspect ratio
  fprintf(fid,'%.3f %.3f %.3f\n',aratio(1),aratio(2),aratio(3));
      
end


fclose(fid);

    

    
    
%----------------------------------
function [fc_w,up_w] = pose2fcpoint(cam_pose)

% ray along principal axis
fc_c = [0;0;1];

T_wc = [rotxyz(cam_pose(1:3)) cam_pose(4:6)];

% put focal point in world coordinates
fc_w = T_wc*[fc_c;1];

% up vector
up_c = [0;-1;0]; % opposite of +Y dir

% from vtkcode on projectedtexture it seems that
% the up vector is oriented in world frame but not translated
up_w = T_wc*[up_c;0];


%--------------------------------------------
function planevec = pose2plane_olds(cam_pose,symdelta)

% planes that define the frustum
% 4 go through camera center and along rays at corners
points = [0 +symdelta(1) +symdelta(1) -symdelta(1) -symdelta(1);
	  0 +symdelta(2) -symdelta(2) -symdelta(2) +symdelta(2);
	  0  1            1            1            1 ] 
	  

T_wc = [rotxyz(cam_pose(1:3)) cam_pose(4:6)];

% transform points into world frame
points_w = T_wc*[points;ones(1,5)];

% calculate planes %Ax+By+Cz+D = 0
planeR = calc_plane(points_w(:,[1 2 3]));
planeL = calc_plane(points_w(:,[1 4 5]));
planeU = calc_plane(points_w(:,[1 3 4]));
planeD = calc_plane(points_w(:,[1 2 5]));
planeN = calc_plane(1.5*points_w(:,[2 3 4]));
planeF = -calc_plane(7*points_w(:,[2 3 4]));

% planevec
planevec = [planeR;planeL;planeU;planeD;planeN;planeF];



%--------------------------------------------
function [points,normals] = pose2planes(cam_pose,symdelta)

% planes that define the frustum
% 4 go through camera center and along rays at corners
points = [0 +symdelta(1) +symdelta(1) -symdelta(1) -symdelta(1)  0;
	  0 +symdelta(2) -symdelta(2) -symdelta(2) +symdelta(2)  0;
	  0.25  1            1            1            1         20 ]; %7
	  

T_wc = [rotxyz(cam_pose(1:3)) cam_pose(4:6)];
%T_wc = [eye(3) zeros(3,1)];
% transform points into world frame
points_w = T_wc*[points;ones(1,6)];

% assume these are rotated but not translated
points_r = T_wc(1:3,1:3)*points;

% calculate vectors
rayRU = points_r(:,3);
rayRD = points_r(:,2);
rayLU = points_r(:,4);
rayLD = points_r(:,5);

% normals out of frustum

nR = - cross(rayRU,rayRD);
nL = - cross(rayLD,rayLU);
nU = - cross(rayLU,rayRU);
nD = - cross(rayRD,rayLD);
nN = - points_r(:,1); % normal near plane
nF = + points_r(:,1); % normal far plane

nR = nR/norm(nR);
nL = nL/norm(nL);
nU = nU/norm(nU);
nD = nD/norm(nD);
nN = nN/norm(nN);
nF = nF/norm(nF);

% planevec
normals = [nN nR nL nU nD nF];

points = points_w(:,[1 2 4 3 2 6]); 





%--------------------------------------------
function plane = calc_plane(points)

M = points';
B = [-1;-1;-1];
plane = M\B;
plane = [plane;1];


