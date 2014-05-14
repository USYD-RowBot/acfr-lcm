function [Ru,tu,costu] = relorient_sample(x1,y1,x2,y2,rph_mean,Cov_rph,fignum)
%function [Rts,cost] = relorient_sample(x1,y1,x2,y2,rph_mean,Cov_rph,fignum)
%   The main idea is to use relorient_horn.m and initialize it many times by
%   sampling around the outer boundary of our orientation prior to see if
%   there are any other nearby local minimum solutions.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-12-2003      rme         Created and written.
%    09-15-2003      rme         Instead of sampling the unit sphere
%                                randomly, I changed the code to call
%                                sphere_tri and sample the unit sphere
%                                using equidistant samples.
%    11-04-2004      rme         Added variable persistence to make code more efficient.
%    11-05-2004      rme         Reorganized R,t arrays to be simpler to access.
%    11-14-2004      rme         Rewrote rph sampling strategy.

Nsamps = 300;

if ~exist('fignum','var')
  fignum = 0;
end

% chi-squared random variable with 3 degrees of freedom associated with
% alpha percent confidence bounds
persistent alpha k2;
if isempty(k2)
  sigma = 3;
  alpha = 1 - 2*normcdf(-sigma);
  k2 = chi2inv(alpha,3);
end

% under the assumption that the orientation prior PDF is described by a
% Gaussian random variable with mean rph_mean and covariance Cov_rph, then
% k2 is Chi-squared random variable and the bounding ellipsoid in parameter
% space associated with the alpha percent confidence level is given by:
% (X-rph_mean)'*inv(Cov_rph)*(X-rph_mean) = k2 where X = [r p h]';

% randomly sample rph space
randn('state',sum(100*clock));
[V,D] = eig(full(Cov_rph));
samples = V*sqrt(D)*randn(3,Nsamps);

% eigenvalue decomposition of the covariance matrix
%[V,D] = eig(full(Cov_rph));
% bounding ellipsoid in eigenvector space, (note ellipsoid generates (N+1)-by-(N+1) matrices)
%N = ceil(sqrt(Nsamps))-1;
%[e1,e2,e3] = ellipsoid(0,0,0,sqrt(k2*D(1,1)),sqrt(k2*D(2,2)),sqrt(k2*D(3,3)),N);
% contruct a 3xNsamps matrix of ellipsoid points
%ellipse3d = [e1(:), e2(:), e3(:)]';
% rotate eigenvector space ellipsoid to rph space ellipsoid
%ellipse3d = V*ellipse3d;

% make sure we also sample at the mean of our distribution by adding the
% origin to our sample set
samples = [samples, [0;0;0]];

Nsamps = size(samples,2);
cost = zeros(1,Nsamps);
t = zeros(3,Nsamps);
R = zeros(3,3,Nsamps);
for ii=1:Nsamps
  % generate a sample from the alpha contour of rph_prior's PDF
  rph_i = rph_mean + samples(:,ii);
  
  % use this orientation as an initial guess in Horn's algorithm
  [Ri,ti,costi] = relorient_horn(x1,y1,x2,y2,rotxyz(rph_i));

  % add the solution to the solution set
  cost(ii)  = costi;
  t(:,ii)   = ti;
  R(:,:,ii) = Ri;
end

% remove duplicate solutions for R,t i.e. Ru,tu are the "unique" solutions
[Ru,tu,costu] = Rt_del_duplicates(R,t,cost,1e-2,1e-1);

if fignum > 0
  %plot_bounding_ellipsoid(ellipse3d,alpha,fignum);
  plot_rph_samples(samples,cost,alpha,fignum);  
  drawnow;
end

%********************************************************************************************
function [Ru,tu,costu] = Rt_del_duplicates(R,t,cost,min_dr,min_dt)
%function [Ru,tu,costu] = Rt_del_duplicates(R,t,cost,min_dr,min_dt)
% delete duplicate entries from list of transformations [R,t]
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%                    B. Triggs   Created and written.
%    09-10-2003      rme         Modified to use dot product norm for t
%                                and use error metric e as well as return
%                                selection index sel.
%    11-05-2004      rme         Reorganized R,t arrays to make access simpler

% initialize unique pose set
tu = t(:,1);
Ru = R(:,:,1);
costu = cost(1);

I = eye(3);
for ii=1:size(R,3)
  duplicate = false;
  % for each Ri,ti pose
  ti = t(:,ii);  Ri = R(:,:,ii); costi = cost(ii);
  for jj=1:size(Ru,3)
    % check against our list of unique poses
    tj = tu(:,jj); Rj = Ru(:,:,jj); costj = costu(jj);
    % compute unit magnitude baseline error
    error_t = 1-ti'*tj;
    if error_t < min_dt
      % compute rotation matrix error
      error_R = norm(Ri'*Rj-I,'inf');
      if error_R < min_dr
	duplicate = true;
	break;
      end
    end
  end % for jj
  
  if ~duplicate
    % add Ri,ti to the unique set
    tu(:,end+1)   = ti;
    Ru(:,:,end+1) = Ri;
    costu(end+1)  = costi;
  elseif costi < costu(jj)
    % replace duplicte R,t entry with lower error pose
    tu(:,jj)   = ti;
    Ru(:,:,jj) = Ri;
    costu(jj)  = costi;
  end
end % for ii


%********************************************************************************************
function plot_bounding_ellipsoid(ellipse3d,alpha,fignum)
figure(fignum); clf;
ellipse3d = ellipse3d*RTOD; % convert from radians to degrees
N = sqrt(size(ellipse3d,2));
%surfl(reshape(ellipse3d(1,:),[N N]), ...
%      reshape(ellipse3d(2,:),[N N]), ...
%      reshape(ellipse3d(3,:),[N N]));
%colormap autumn; shading interp;
xlabel('r [deg]'); ylabel('p [deg]'); zlabel('h [deg]');
title(sprintf(['Relative Orientation Prior (Zero-mean)\n' ...
	       '%d samples of %g%% confidence interval'],N*N,alpha*100));
%hold on;
plot3(ellipse3d(1,:),ellipse3d(2,:),ellipse3d(3,:),'b.');
%hold off;
axis equal; grid on; rotate3d on;

%********************************************************************************************
function plot_rph_samples(samples,cost,alpha,fignum)
figure(fignum); clf;
samples = samples*RTOD; % convert from radians to degrees
Nsamps = size(samples,2);
fscatter3(samples(1,:),samples(2,:),samples(3,:),log10(cost),jet(256));
axis equal; grid on; view(3); rotate3d on;
xlabel('r [deg]'); ylabel('p [deg]'); zlabel('h [deg]');
title(sprintf(['Relative Orientation Prior (Zero-mean)\n' ...
	       '%d samples of %g%% confidence interval'],Nsamps,alpha*100));
