function [fni,fnj,probability,mu_d,sigma_dd] = link_hypothesis_ekf(TheConfig)
%function [fni,fnj,probability,mu_d,sigma_dd] = link_hypothesis_ekf(TheConfig)  
%
% OUTPUT:
% fni,fnj feature indexes of hypothesized overlapping image pairs
%         note that fni < fnj always
% bmag,bsigma baseline magnitude and standard deviation
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    02-27-2004      rme         Created from linkhypoth_manual.m
%    03-30-2004      rme         Modified to calculate 1st order
%                                covariance associated with Euclidean
%                                distance.  Also changed to use
%                                TheBathy altitude information.
%    04-08-2004      rme         Updated input/output arguments.
%    09-09-2004      rme         Updated to extract the Xp_i pose elements from Xf_ii
%                                Limited max number of link candidates
%    10-28-2004      rme         Major code reorganization.
%    11-02-2004      rme         Updated calculation of testlinks.
%    11-04-2004      rme         Updated probability calculation.
%    11-05-2004      rme         Renamed to link_hypothesis_ekf.m
%    12-20-2004      rme         Moved Bathy structure out from under the TheJournal
%                                and into a new global variable TheBathy  

global TheJournal TheBathy;

persistent Distance;
if isempty(Distance)
  Distance.d   = TheJournal.Links.plinks;
  Distance.std = TheJournal.Links.plinks;
end

% define thresholds
minPercentOverlap = 0.25; % percent image overlap [0 1]
maxPercentOverlap = 0.90; % max is used to define a minimum baseline distance to image
			  % registration is more robust to small translation/rotation errors
minDistanceLowerLimit = 0.20; % [meters]
confidence = 0.95; % confidence in overlap [0 1]


% one-dimensional image width for unit altitude
FOV = 40*DTOR;
normWidth = 2*tan(FOV/2);

% static pose of camera w.r.t vehicle
x_vc = TheConfig.SensorXform.PXF.x_vs;

% shorthand index
Ni   = find(TheJournal.Index.featureLUT > 0); 
Ni   = Ni(1);                 % index of first camera feature
Nf   = TheJournal.Index.Nf;   % index of last camera feature
Xp_i = TheJournal.Index.Xp_i; % pose elements index


% * plinks is an upper triangular [Nf x Nf] array of 0's and 1's where a 1
%   represents a hypothesized correspondence between feature i and j
% * vlinks is also an upper triangular [Nf x Nf] array.
%   0 means a correspondence between i and j has not been tried
%   1 means a correspondence between i and j has been established
%  -N means a correspondence between i and j has been tried N times and
%     has failed each time.
TheJournal.Links.plinks(1:Nf,1:Nf) = 0;

Distance.d(1:Nf,1:Nf)   = 0;
Distance.std(1:Nf,1:Nf) = 0;

% compute Euclidean horizontal distance between camera centers
% and use it as a metric for hypothesizing overlapping image pairs
for ii=Ni:Nf
  % vehicle pose and covariance in local-level frame
  Xf_i      = TheJournal.Index.Xf_ii{ii}(Xp_i);
  xi_lv     = TheJournal.Ekf.mu(Xf_i);
  Cov_xi_lv = TheJournal.Ekf.Sigma(Xf_i,Xf_i);

  % camera pose in local-level frame
  [xi_lc,Ji_plus] = head2tail(xi_lv,x_vc);

  for jj=ii+1:Nf
    if TheJournal.Links.vlinks(ii,jj) ~= 0
      % skip this image pair altogether since either:
      % 1) vlinks < 0 previous reg attempt failed
      % 2) vlinks = 1 image pair has already been successfully registered
      continue;
    end
    
    % vehicle pose, covariance, and cross-covariance in local-level frame
    Xf_j        = TheJournal.Index.Xf_ii{jj}(Xp_i);
    xj_lv       = TheJournal.Ekf.mu(Xf_j);
    Cov_xj_lv   = TheJournal.Ekf.Sigma(Xf_j,Xf_j);
    Cov_xixj_lv = TheJournal.Ekf.Sigma(Xf_i,Xf_j);
    
    % camera pose in local-level frame
    [xj_lc,Jj_plus] = head2tail(xj_lv,x_vc);
    
    % horizontal distance between camera centers
    [mu_d,J_ij] = euclideanDistance(xi_lc(1:2),xj_lc(1:2));
    
    % total Jacobian w.r.t vehicle poses xi_lv xj_lv
    J = [J_ij(1:2)*Ji_plus(1:2,1:6), J_ij(3:4)*Jj_plus(1:2,1:6)];
    
    % 1st order covariance of horizontal distance
    Cov_p = [Cov_xi_lv,    Cov_xixj_lv;
	     Cov_xixj_lv', Cov_xj_lv];
    Sigma_dd = J*Cov_p*J';
    std_dd = sqrt(Sigma_dd);
    
    % max altitude for camera pair
    alt = max([TheBathy(ii).alt; TheBathy(jj).alt]);
    
    % one dimensional image width
    imageWidth = alt*normWidth;
    maxDistance = (1-minPercentOverlap)*imageWidth;
    minDistance = (1-maxPercentOverlap)*imageWidth;
    minDistance = max(minDistance,minDistanceLowerLimit);
    
    % probability that the horizontal distance falls withing the range:
    % min_distance < d < (1-POVERLAP)*altitude
    probability = normcdf(maxDistance, mu_d, std_dd) - normcdf(minDistance, mu_d, std_dd);

    % store computations
    Distance.d(ii,jj) = mu_d;
    Distance.std(ii,jj) = std_dd;
    TheJournal.Links.plinks(ii,jj) = probability;

  end % for jj

end % for ii

% upper triangular matrix of hypothesized overlaping image pairs
testlinks = TheJournal.Links.plinks;

% candidate feature pairs which should have overlap with the given confidence
[fni,fnj] = find(testlinks > confidence); % i < j always because testlinks is upper triangular
if isempty(fni)
  % set output arguments to empty
  fni = [];  fnj = []; probability = [];  mu_d = [];  sigma_dd = [];
  return;
end
ii = sub2ind(size(testlinks),fni,fnj);
probability = full(testlinks(ii));
mu_d        = full(Distance.d(ii));
sigma_dd    = full(Distance.std(ii));

% sort returns in ascending order
[probability,ii] = sort(probability);
% reorganize into descending order
probability = probability(end:-1:1);
ii  = ii(end:-1:1);
fni = fni(ii);
fnj = fnj(ii);
mu_d = mu_d(ii);
sigma_dd = sigma_dd(ii);

% keep only the K most likey candidate pairs
kk = min(length(fni), TheConfig.Estimator.max_link_hypoth);
fni = fni(1:kk);
fnj = fnj(1:kk);
probability = probability(1:kk);
mu_d = mu_d(1:kk);
sigma_dd = sigma_dd(1:kk);

% plot links
if TheConfig.Plot.plinks
  figure(2000); imagesc(TheJournal.Links.plinks(1:Nf,1:Nf),[0 1]);
  colormap cool; colorbar; axis square; title('plinks probability');
end
if TheConfig.Plot.vlinks
  figure(2001); imagesc(TheJournal.Links.vlinks(1:Nf,1:Nf),[-1 1]);
  colormap(flipud(cool)); colorbar; axis square; title('vlinks');
end
