function ncam = cameras(pose,K,res,filename)
% generates camera poses for vtk from matlab poses

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
fid = fopen(filename,'w');

k = 10;


[fc_w,up_w] = pose2fcpoint(pose(:,k));

fprintf(fid,'vtkProjectedTexture cam%i\n',k);
fprintf(fid,'cam%i SetPosition ',k);
fprintf(fid,'%.3f %.3f %.3f\n',pose(4,k),pose(5,k),pose(6,k));

fprintf(fid,'cam%i SetFocalPoint ',k);
fprintf(fid,'%.3f %.3f %.3f\n',fc_w(1),fc_w(2),fc_w(3));

fprintf(fid,'cam%i SetUp ',k);
fprintf(fid,'%.3f %.3f %.3f\n',up_w(1),up_w(2),up_w(3));

fprintf(fid,'cam%i SetAspectRatio ',k);
fprintf(fid,'%.3f %.3f %.3f\n',aratio(1),aratio(2),aratio(3));


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
