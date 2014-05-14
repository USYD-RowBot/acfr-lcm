function [fni,fnj,probability,mu_d,sigma_dd] = link_hypothesis(TheConfig);
%function [fni,fnj,probability,mu_d,sigma_dd] = link_hypothesis(TheConfig);
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
%    11-11-2004      rme         Renamed to link_hypothesis.m and included
%                                extract_poses_ekf.m & exract_poses_eif.m
%    12-20-2004      rme         Moved Bathy structure out from under the TheJournal
%                                and into a new global variable TheBathy

global TheJournal TheBathy;

persistent Distance;
if isempty(Distance);
  Distance.d   = TheJournal.Links.plinks;
  Distance.std = TheJournal.Links.plinks;
end;

% define thresholds
minPercentOverlap = 0.25; % percent image overlap [0 1]
maxPercentOverlap = 0.95; % max is used to define a minimum baseline distance to image
			  % registration is more robust to small translation/rotation errors
minDistanceLowerLimit = 0.20; % [meters]
confidence = 0.95; % confidence in overlap [0 1]


% one-dimensional image width for unit altitude
FOV = 40*DTOR;
normWidth = 2*tan(FOV/2);

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
%for fni=Ni:Nf;
for fni=Ni:Nf-1; % just check for newest image
  outerLoop = true;
%  for fnj=fni+1:Nf;
  for fnj=Nf;
    if TheJournal.Links.vlinks(fni,fnj) ~= 0;
      % skip this image pair altogether since either:
      % 1) vlinks < 0 previous reg attempt failed
      % 2) vlinks = 1 image pair has already been successfully registered
      continue;
    end;
    
    % extract pose information from our state estimate
    [x_lvi,x_lvj,x_vc,Sigma] = extract_poses(fni,fnj,TheConfig);
    
    if outerLoop;
      % camera pose i in local-level frame
      [x_lci,Ji_plus] = head2tail(x_lvi,x_vc);
    end;

    % camera pose j in local-level frame
    [x_lcj,Jj_plus] = head2tail(x_lvj,x_vc);
    
    % horizontal distance between camera centers
    [mu_d,J_ij] = euclideanDistance(x_lci(1:2),x_lcj(1:2));
    
    % total Jacobian w.r.t x_lvi, x_lvj, and x_vc via Chain-rule
    % i.e. d(mu_d)/d(x_lci,xlcj) * d(x_lci,xlcj)/d(x_lvi,x_lvj,x_vc)
    J = J_ij * [Ji_plus(1:2,1:6), zeros(2,6),       Ji_plus(1:2,7:12);
		zeros(2,6),       Jj_plus(1:2,1:6), Jj_plus(1:2,7:12)];
    
    % 1st order covariance of horizontal distance
    Sigma_dd = J*Sigma*J';
    std_dd = sqrt(Sigma_dd);
    
    % max altitude for camera pair
    alt = prctile([TheBathy(fni).alt; TheBathy(fnj).alt],90);
    
    % one dimensional image width
    imageWidth  = alt*normWidth;
    maxDistance = (1-minPercentOverlap)*imageWidth;
    minDistance = (1-maxPercentOverlap)*imageWidth;
    minDistance = max(minDistance,minDistanceLowerLimit);
    
    % probability that the horizontal distance falls withing the range:
    % minDistance < d < maxDistance
    probability = normcdf(maxDistance, mu_d, std_dd) - normcdf(minDistance, mu_d, std_dd);

    % store computations
    Distance.d(fni,fnj) = mu_d;
    Distance.std(fni,fnj) = std_dd;
    TheJournal.Links.plinks(fni,fnj) = probability;

    %[probability,minDistance,mu_d,maxDistance,std_dd,alt,fni,fnj]
    
    outerLoop = false;
  end; % for fnj

end; % for fni

% upper triangular matrix of hypothesized overlaping image pairs
testlinks = TheJournal.Links.plinks;

% candidate feature pairs which should have overlap with the given confidence
[fni,fnj] = find(testlinks > confidence); % i < j always because testlinks is upper triangular
if isempty(fni);
  % set output arguments to empty
  [fni,fnj,probability,mu_d,sigma_dd] = deal([]);
  return;
end;
ii = sub2ind(size(testlinks),fni,fnj);
probability = full(testlinks(ii));
mu_d        = full(Distance.d(ii));
sigma_dd    = full(Distance.std(ii));

% sort based upon probability and then minimum distance
[junk,ii] = sortrows([-probability,mu_d]);
fni = fni(ii);
fnj = fnj(ii);
mu_d = mu_d(ii);
sigma_dd = sigma_dd(ii);
probability = probability(ii);

% keep only the K most likey candidate pairs
kk = min(length(fni), TheConfig.Estimator.max_link_hypoth);
fni = fni(1:kk);
fnj = fnj(1:kk);
probability = probability(1:kk);
mu_d = mu_d(1:kk);
sigma_dd = sigma_dd(1:kk);

% plot links
if TheConfig.Plot.plinks;
  figure(70); imagesc(TheJournal.Links.plinks(1:Nf,1:Nf),[0 1]);
  colormap cool; colorbar; axis image; title('plinks probability');
  set(70,'Name','Proposed Links');
end;
if TheConfig.Plot.vlinks;
  figure(71); imagesc(TheJournal.Links.vlinks(1:Nf,1:Nf),[-1 1]);
  colormap(flipud(cool)); colorbar; axis image; title('vlinks');
  set(71,'Name','Verified Links');
end;
