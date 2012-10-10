function ncam = cameras2(pose,K,res,filename)
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

for k = [10 11];

  [fc_w,up_w] = pose2fcpoint(pose(:,k));
  %planevec = pose2planes(pose(:,k),symdelta);
  [points,normals] = pose2planes(pose(:,k),symdelta);
  
  % create points and normals to describe planes for clipping
  fprintf(fid,'vtkPoints points%i\n',k);
  for i = 1:6
    fprintf(fid,'   points%i InsertNextPoint %.3f %.3f %.3f\n',k,points(1,i),points(2,i),points(3,i));
  end 
  fprintf(fid,'vtkFloatArray normals%i\n',k);
  fprintf(fid,'   normals%i SetNumberOfComponents %i\n',k,3);
  for i = 1:6
    fprintf(fid,'   normals%i InsertNextTuple3 %.3f %.3f %.3f\n',k,normals(1,i),normals(2,i),normals(3,i));
  end
  
  
  % clipping for that projected texture  
  fprintf(fid,'vtkPlanes planes%i\n',k);
  fprintf(fid,'   planes%i SetPoints points%i\n',k,k);
  fprintf(fid,'   planes%i SetNormals normals%i\n',k,k);
  %fprintf(fid,'   planes%i SetFrustumPlanes ',k);
  %fprintf(fid,'%.4f ',planevec);
  %fprintf(fid,'\n');

  % read texture
  fprintf(fid,'vtkPNGReader pngReader%i\n',k);
  fprintf(fid,'   pngReader%i SetFileName ',k);
  fprintf(fid,'"/files1/data/bermuda02/Images/i20020827/20020827_2149.navsurv/RadComp/test00%i.png"\n',k);
  fprintf(fid,'vtkTexture tex%i\n',k);
  fprintf(fid,'   tex%i SetInput [pngReader%i GetOutput]\n',k,k);
  fprintf(fid,'   tex%i SetRepeat 0\n',k);
  
  fprintf(fid,'vtkClipPolyData clipper%i\n',k);
  fprintf(fid,'   clipper%i SetInput [meshNormals GetOutput]\n',k);
  fprintf(fid,'   clipper%i SetClipFunction planes%i\n',k,k);
  fprintf(fid,'   clipper%i InsideOutOn\n',k);
  
  % projected texture
  fprintf(fid,'vtkProjectedTexture cam%i\n',k);
  fprintf(fid,'   cam%i SetPosition ',k);
  fprintf(fid,'%.3f %.3f %.3f\n',pose(4,k),pose(5,k),pose(6,k));

  fprintf(fid,'   cam%i SetFocalPoint ',k);
  fprintf(fid,'%.3f %.3f %.3f\n',fc_w(1),fc_w(2),fc_w(3));
  
  fprintf(fid,'   cam%i SetUp ',k);
  fprintf(fid,'%.3f %.3f %.3f\n',up_w(1),up_w(2),up_w(3));

  fprintf(fid,'   cam%i SetAspectRatio ',k);
  fprintf(fid,'%.3f %.3f %.3f\n',aratio(1),aratio(2),aratio(3));
  fprintf(fid,'   cam%i SetInput [clipper%i GetOutput]\n',k,k);
  
  
  
  fprintf(fid,'vtkPolyDataMapper clip%iMapper\n',k);
  fprintf(fid,'   clip%iMapper SetInput [cam%i GetOutput]\n',k,k);
  fprintf(fid,'vtkActor clip%iActor\n',k);
  fprintf(fid,'   clip%iActor SetMapper clip%iMapper\n',k,k);
  fprintf(fid,'   clip%iActor SetTexture tex%i\n',k,k);
  
  
  % add actor to renderer
  fprintf(fid,'ren1 AddActor clip%iActor\n',k);
  
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
	  1.5  1            1            1            1          7 ] 
	  

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


