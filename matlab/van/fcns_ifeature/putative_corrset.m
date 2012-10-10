function [sel1,sel2,simA,simB,cmatrix] = ...
    putative_corrset(u1, v1, u2, v2,  Z1, Z2, Cov_Z1, Cov_Z2, smatrix, minmaxString, ...
		     X_c1c2, X_c2c1, P_c1c2, P_c2c1, Image1, Image2, TheConfig);

%function [sel1,sel2,simA,simB,cmatrix] = ...
%    putative_corrset(u1, v1, u2, v2,  Z1, Z2, Cov_Z1, Cov_Z2, smatrix, minmaxString, ...
%		     X_c1c2, X_c2c1, P_c1c2, P_c2c1, Image1, Image2, TheConfig);
%
%   INPUT
%   u1,v1,u2,v2: [Mx1]/[Nx1] distortion free interest from images I1,I2 respectively
%         Z1,Z2: [Mx1]/[Nx1] scene depth prior for each feature point in camera frame
% Cov_Z1,Cov_Z2: [Mx1]/[Nx1] covariance of scene depth prior
%       smatrix: [MxN] feature similarity matrix
%  minmaxString: {'min','max'}  choose correspondeces to minimize or maximize smatrix
%        X_c1c2: [6x1] pose of camera 2 w.r.t. camera 1
%        X_c2c1: [6x1] pose of camera 1 w.r.t. camera 2
%        P_c1c2: [6x6] covariance matrix of X_c1c2
%        P_c2c1: [6x6] covariance matrix of X_c2c1
% Image1,Image2: image data structures
%     TheConfig: configuration data structure
%
%   OUTPUT
%   sel1, sel2: indices of putative correspondences
%         simA: similarity score of match
%         simB: similarity score of next closet match
%      cmatrix: (optional) pose constrained correspondence matrix
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-03-2003      rme         Created.
%    04-01-2004      rme         Changed TheConfig to include Calib
%    04-13-2004      rme         Changed rcpose_t args to pose vectors
%    04-15-2004      rme         Rewrote putative correspondence function
%                                and unique feature function.
%    12-18-2004      rme         Changed input arguments and modified to
%                                be more generic.
%    12-22-2004      rme         Modified to use relview_ptxfer2.m
%    12-25-2004      rme         Modified to pass chiSquare2dof as an input argument instead
%                                of confidence level alpha.  This makes it easier to
%                                scale the bounding ellipse size across multiple functions.

% initialize ouput arguments
[sel1,sel2,simA,simB] = deal([]);

% assign pointers
Calib = TheConfig.Calib;

%======================================================
% RESTRICT FEATURE CORRESPONDENCE BASED UPON PRIOR POSE
%======================================================  
% transfer feature points from image 1 into image 2 based upon
% scene depth and pose prior information
%------------------------------------------------------  
% predicted location of feature points (u1,v1) in image 2
% NOTE that i'm assigning *ZERO* uncertainty for the camera calibration
% matrix, i am treating it as a known quantity
%[u2p,v2p,Cov_u2pv2p] = relview_ptxfer(Calib.K, X_c2c1, Z1, u1, v1, P_c2c1, Cov_Z1);
[u2p,v2p,Cov_u2pv2p] = relview_ptxfer2(Calib.K, X_c2c1, Z1, u1, v1, P_c2c1, Cov_Z1(1));
% predicted location of feature points (u2,v2) in image 1
%[u1p,v1p,Cov_u1pv1p] = relview_ptxfer(Calib.K, X_c1c2, Z2, u2, v2, P_c1c2, Cov_Z2);
[u1p,v1p,Cov_u1pv1p] = relview_ptxfer2(Calib.K, X_c1c2, Z2, u2, v2, P_c1c2, Cov_Z2(1));
% specify confidence level alpha
alpha = 1-2*normcdf(-6);
fudge = 1;
chiSquare2dof = chi2inv(alpha,2) * fudge^2;
% compute a pose restricted correspondence matrix  from image 1 into image 2
% and "AND" it with a matrix from image 2 to image 1 since a consistent
% forwards and backwards mapping requires the intersection
%----------------------------------------------------------------------------
cmatrix = pose_restricted_cmatrix(u2,v2,u2p,v2p,Cov_u2pv2p,chiSquare2dof) ...
        & transpose(pose_restricted_cmatrix(u1,v1,u1p,v1p,Cov_u1pv1p,chiSquare2dof));


% plot pose bounded search regions
if TheConfig.Plot.twoview_pt_xfer;
  % fundamental matrix based upon prior pose
  %------------------------------------------------------
  % prior-pose fundamental matrix satisfying u2'*F*u1 = 0
  t = X_c2c1(1:3);
  R = rotxyz(X_c2c1(4:6));
  Fpp21 = F_from_KRt(R,t,Calib.K);
  private_plot_search_regions(cmatrix,Image1,Image2,u1,v1,u2,v2,chiSquare2dof, ...
			      u1p,v1p,Cov_u1pv1p,u2p,v2p,Cov_u2pv2p,Fpp21);
end;

% check if any pose restricted candidate correspondences even exist
if ~any(cmatrix(:)); return; end;

%==========================================================================
% ASSIGN PUTATIVE CORRESONDENCES BASED UPON POSE PRIOR AND FEATURE VECTOR
% SIMILARITY SCORE
%==========================================================================
% pose contrained similarity matches
[sel1,sel2,simA,simB] = private_simmatch(cmatrix,smatrix,minmaxString);


%************************************************************************************************
function [sel1,sel2,simA,simB] = private_simmatch(cmatrix,smatrix,minmaxString);
% INPUTS:
%   cmatrix is a [M x N] correspondence matrix where M is the number of
%   features in image I1 and N is the number of features in image I2.  A
%   nonzero entry in cmatrix(i,j) indicates a possible pose correspondence
%   between features Mi and Nj.
%
%   smatrix is a [M x N] similarity matrix where smatrix(i,j) is the
%   feature similarity score between feature Mi and Nj.
%
% OUTPUTS:
%   sel1, sel2: are [L x 1] vectors defining putative feature correspondences
%         simA: is the similarity score of match
%         simB: is the similarity score of next closet match

% initialize output arguments
[sel1,sel2,simA,simB] = deal([]);
[M,N] = size(smatrix);

switch lower(minmaxString);
case 'max'; maximize = true;
case 'min'; maximize = false;
otherwise; error('unknown value for minmaxString');
end;

% reject similarity scores which are not consistent with the pose constraint
if maximize;
  smatrix(~cmatrix) = 0;
  smatrix = sparse(smatrix);
else;
  smatrix(~cmatrix) = inf;
end;

% best matches in I2 for the features given in I1
if maximize;
  [score12,fwd12] = max(smatrix,[],2);
else;
  [score12,fwd12] = min(smatrix,[],2);
end;

% best matches in I1 for the features given in I2
if maximize;
  [score21,fwd21] = max(smatrix,[],1);
else;
  [score21,fwd21] = min(smatrix,[],1);
end;
% make column vectors
fwd21   = fwd21';
score21 = score21'; 

% find the forward (i.e. 1->2) and backward (i.e. 2->1) feature mappings
% which are consistent with each other
sel2 = find(fwd12(fwd21) == [1:N]');
sel1 = fwd21(sel2);
simA = score21(sel2); % similarity score of match

% by default the max()/min() function will return the index 1 for
% the rows and columns of smatrix which are all zeros.  this
% can lead to the putative correspondence assignment of (1,1).
% check for this condition and remove it if it exists.
if smatrix(sel1(1),sel2(1)) == 0;
  sel1(1) = [];
  sel2(1) = [];
  simA(1) = [];
end;

% find similarity score of next best match
smatrix(sub2ind([M,N],sel1,sel2)) = nan;
if maximize;
  simB = max(smatrix(sel1,:),[],2);
else;
  simB = min(smatrix(sel1,:),[],2);
end;

simA = full(simA);
simB = full(simB);

%***********************************************************************************
function private_plot_search_regions(cmatrix,Image1,Image2,u1,v1,u2,v2,chiSquare2dof, ...
				     u1p,v1p,Cov_u1pv1p,u2p,v2p,Cov_u2pv2p,Fpp21);
% plot pose bounded search region when transfering points (u1,v1) to (u2p,v2p)
figure(30);
sample_ellipses(Image1.Iwarp,Image2.Iwarp,u1,v1,u2,v2,u2p,v2p,Cov_u2pv2p,Fpp21,chiSquare2dof);
figure(30);
title('Pose Prior Ellipse I1 with sampling of (u1,v1)');
set(30,'Name','Pose Prior Corr Ellipse I1');
figure(31); 
title('Pose Prior Ellipse I2 with sampling of (u2p,v2p)');
set(31,'Name','Pose Prior Corr Ellipse I2');
% plot pose bounded search region when transfering points (u2,v2) to (u1p,v1p)
figure(32);
sample_ellipses(Image2.Iwarp,Image1.Iwarp,u2,v2,u1,v1,u1p,v1p,Cov_u1pv1p,Fpp21',chiSquare2dof);
figure(32); 
title('Pose Prior Ellipse I2 with sampling of (u2,v2)');
set(32,'Name','Pose Prior Corr Ellipse I2');
figure(33);
title('Pose Prior Ellipse I1 with sampling of (u1p,v1p)');
set(33,'Name','Pose Prior Corr Ellipse I1');

figure(34);
spy(cmatrix); axis equal; axis tight; 
grid on; set(gca,'GridLineStyle',':');
ylabel('I1');
xlabel('I2');
title(sprintf('cmatrix %.2f%% non-zero',density(cmatrix)*100));
set(34,'Name','cmatrix');


%***********************************************************************************    
function private_plot_putative(label,Calib,Image,Features,psel,usel,fignum);
figure(fignum);
imagesc(Calib.udata,Calib.vdata,Image.Iwarp); colormap gray;
axis equal; axis tight;

hold on;
% plot unique matches
plot_multicolor(Features.Zernike.uc(psel),Features.Zernike.vc(psel),'+');
plot_multicolor(Features.Zernike.uc(psel),Features.Zernike.vc(psel),'o');

% plot non-unique matches
%plot_multicolor(Features.uc(usel),Features.vc(usel),'^');
plot_multicolor(Features.Zernike.uc(usel),Features.Zernike.vc(usel),'x');

hold off;
title_str = sprintf('%s Putative Correspondence Set\nUnique: %d  Non-Unique: %d',...
		    label,length(psel),length(usel));
title(title_str);
