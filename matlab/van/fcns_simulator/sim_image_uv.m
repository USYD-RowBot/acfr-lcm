function [Features,tag_t,xpond_t,clinks] = sim_image_uv(Xmat,tag,posemat,K,res,Tvc)
%SIM_IMAGE_UV simulated scene image points given a set of camera poses.
%   [FEAT_T,TAG_T,XPOND_T,CLINKS] = SIM_IMAGE_UV(XMAT,TAG,POSEMAT,K,RES,Tvc) returns a
%   image feature structure FEAT_T indexed by camera and containing u,v
%   pixel coordinates of viewable scene features as well as a tag index
%   indicating which 3D feature the u,v coordinates belong to.  TAG_T is a
%   structure indexed by 3D feature with TAG_T.Iuv a [3 x L] matrix
%   specifying [image, u, v] for each visible image.  CLINKS is a [M x M]
%   camera links array, i.e. CLINKS(i,j)=1 indicates that images i and j
%   share common features.  XPOND_T is a correspondence structure
%   providing a mapping between common feature points between images i
%   and j, i.e. 
%   FEAT_T(i).u(XPOND_T(i,j).seli) <--> FEAT_T(j).u(XPOND_T(i,j).selj)
%
%   Input arguments are:  
%    XMAT : is a [3 x N] [X,Y,Z] matrix of 3D scene features where N is
%           the number of 3D features.  
%    TAG : is a [N x 1] label of scene features.
%    POSEMAT : is a [6 x M] [x,y,z,r,p,h] matrix of camera poses where M
%              is the number of cameras.
%    K : is the [3 x 3] camera calibration matrix.
%    RES : is the [resx, resy] image pixel resolution
%    Tvc : (optional) is the [4 x 4] homogenous camera to vehicle
%          transform.  This allows POSEMAT to specify vehicle pose
%          instead of camera pose.  
%
%-----------------------------------------------------------------
%    History:
%    Date          Who        What
%    -----------   -------    -----------------------------
%    20031118      OP         Created and written.
%    20031120      OP         Added measurement noise.
%    20031219      rme        Ryanized

if ~exist('Tvc','var')
  Tvc = eye(4);
end

resx = res(1);
resy = res(2);

Nc = size(posemat,2); % number of camera poses
Nf = size(Xmat,1);    % number of 3D scene features

Features(Nc,1) = struct('u',[],'v',[],'tag',[]);
tag_t(Nf,1)  = struct('Iuv',[]);
for ii = 1:Nc
  % camera pose in world frame
  R = rotxyz(posemat(4:6,ii));
  t = posemat(1:3,ii);
  
  % coordinate transforms
  Twv = [R t; 0 0 0 1]; % vehicle to world xform
  Twc = Twv*Tvc;        % camera  to world xform
  Tcw = inv(Twc);       % world to camera xform
  
  % camera projection matrix
  R = Tcw(1:3,1:3);
  t = Tcw(1:3,4);
  P = K*[R t];
  
  % project scene
  [u,v,inview] = project(P,Xmat,resx,resy);
  
  % keep points which are in view
  sel = find(inview);
  Nviz = length(sel); % number of visible features
  Features(ii).u = u(sel);
  Features(ii).v = v(sel);
  Features(ii).tag = tag(sel);  
  
  % keep track of images of 3D features
  % note that tag_t.Iuv will have the cameras in increasing order
  for kk = 1:Nviz
    tag_t(tag(sel(kk))).Iuv = [tag_t(tag(sel(kk))).Iuv, [ii; u(sel(kk)); v(sel(kk))]];
  end % for kk
  
end % for ii


% now that we've projected the visible portion of the scene into each
% camera, we must now establish correspondences between image feature
% points
[xpond_t,clinks] = gen_xpond(Features);

%-------------------------------------------------------------------
function [u,v,inview] = project(P,Xmat,resx,resy)
% projects 3D points Xmat using camera projection matrix P
U = P*[Xmat; ones(1,size(Xmat,2))];
u = (U(1,:)./U(3,:))';
v = (U(2,:)./U(3,:))';
inview = (u >= 0) & (u <= resx-1) & ...
	 (v >= 0) & (v <= resy-1);

%-------------------------------------------------------------------
function [xpond_t,clinks] = gen_xpond(Features)
% calculates correspondence information for each overlapping image pair (i,j)
% INPUT 
% Features
%
% OUTPUT
% xpond_t is a two-dimensional structure indexed by image numbers (i,j)
%         and contains correspondence indicies which map features between images
%         i and j.
%           matchmat(i,j).seli
%           matchmat(i,j).selj
% clinks  is an upper triangular [M x M] camera links matrix,
%         i.e. clinks(i,j) = 1 if images i and j share common features and M is
%         the number of cameras.
%
%-----------------------------------------------------------------
%    History:
%    Date          Who        What
%    -----------   -------    -----------------------------
%    20031118      OP         Created and written.
%    20031119      OP         Simplified output, eliminated
%                             intermediate structures
%    20031222      rme        Ryanized

% number of cameras
M = length(Features);

% allocate camera links matrix
clinks = false(ncam,ncam);

% loop through and find image projects of common 3D scene features
for ii = 1:M
  % 3D feature labels of camera ii
  tagi = Features(ii).tag;
  for jj = ii+1:M
    % 3D feature labels of camera jj
    tagj = Features(jj).tag;
    [tagcommon,seli,selj] = intersect(tagi,tagj);
    % update clinks and xpond_t if there are common elements
    if ~isempty(tagcommon)
      xpond_t(ii,jj).seli = seli;
      xpond_t(ii,jj).selj = selj;
      clinks(ii,jj) = true;
    end
  end % for jj
end % for ii
