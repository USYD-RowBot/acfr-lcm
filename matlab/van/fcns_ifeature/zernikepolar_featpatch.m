function [patchmat,sel] = zernikepolar_featpatch(Iraw,uo,vo,Zbasis,TformRadial,K,R,w);
%function [patchmat,sel] = zernikepolar_featpatch(Iraw,uo,vo,Zbasis,TformRadial,K,R,w);
%  Input Args:
%  Iraw is the raw gray-level image.
%  [uo,vo] are the feature point locations with top left defined to be (0,0).
%  Zbasis is a Zernike moments data structure as returned by zernikeBasisFcns.
%  TformRadial is a Matlab tform structure which removes radial distortion.
%  K is the camera calibration matrix.
%  R is rotation matrix decribing the camera orientation in some global frame.
%  w is the half size of the feature patch, i.e.  [(2w+1) x (2w+1)].
%
%  Output Args:
%  patchmat is a [len x Nf] matrix of vectorized polar coodinate feature patches which
%  have been detrended.  sel is a selection index of feature points for which a valid
%  patch could be extracted; length(sel) = Nf.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-13-2004      rme         Created from normcorr_featpatch.m
%    12-16-2004      rme         Return detrended polar image patches.

Iraw = double(Iraw);
interpMethod = 'linear'; %{'nearest','linear','cubic'}

%=================================================================
% KEEP FEATURES FOR WHICH WE CAN EXTRACT A VALID PATCH
%=================================================================
% worst case rotation is 45 degrees, therefore keep features around
% image border within this constraint
b = ceil(sqrt(2)*w);
[nr,nc] = size(Iraw);
sel = find((uo>b) & (uo<nc-b) & (vo>b) & (vo<nr-b));
uo = uo(sel); vo = vo(sel);

%=================================================================
% CREATE MATLAB TFORM ARRAY
%=================================================================
% compose homography at infinity based upon camera pose
Hinf = homog_Hinf(K,R);

% create a matlab tform structure for Hinf
% note matlab uses post vs. pre multiply
TformHinf = maketform('projective',Hinf');

% create a composite matlab tfrom structure which implements
% the radial distortion correction followed by the warping of the 
% homography at infinity
TformComposite = maketform('composite',TformHinf,TformRadial);

%=================================================================
% SETUP FOR FEATURE PATCH EXTRACTION
%=================================================================
% calculate location of feature points in warped image
tmp = tformfwd([uo,vo],TformComposite);
[uw,vw] = deal(tmp(:,1),tmp(:,2));

% precalculate sample grid where w is the half size of feature patch
% (i.e. feature patch is (2w+1)x(2w+1))
% note: Zbasis.xsamp, Zbasis.ysamp nonuniformly spaced Cartesian sample
% points corresponding to uniform Polar sample points 
% Zbasis.rsamp, Zbasis.tsamp in the range 0 <= rho <= 1 , 0 <= theta < 2pi
% therefore, scale xgrid,ygrid by half size w to sample desired feature patch size.
[ugrid,vgrid] = deal(Zbasis.xsamp*w , -Zbasis.ysamp*w );

% create a resampler structure
R = makeresampler(interpMethod,'fill');

% pre-allocate memory to hold vectorized feature patches
Nf = length(sel);
patchmat = zeros(Zbasis.size(1),Nf);

%=================================================================
% EXTRACT FEATURE PATCHES
%=================================================================
% calculate each pixel's contribution to the area integral
weight = Zbasis.darea / sum(Zbasis.darea(:));
for ii=1:Nf;
  % calculate sample points in warped image space
  [uw_samp,vw_samp] = deal(uw(ii)+ugrid, vw(ii)+vgrid);
  
  % map warped image sample points to raw image space
  tmp = tforminv(TformComposite,[uw_samp(:),vw_samp(:)]);
  [uo_samp,vo_samp] = deal(reshape(tmp(:,1),size(uw_samp)), reshape(tmp(:,2),size(vw_samp)));

  % sample the raw image directly by specifying the raw image space sample points
  % with a tmap_b data structure used within the tformarray command
  tmap_b = cat(3,uo_samp+1,vo_samp+1); % account for Matlab array indexing
  polarpatch = tformarray(Iraw,[],R,[2 1],[1 2],[],tmap_b,0);
  
  % de-mean, normalize, vectorize, and store
  patchmat(:,ii) = zernikeDetrendPolar(polarpatch(:),Zbasis);
  
end; % for ii=1:Nf
