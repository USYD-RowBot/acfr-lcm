function [fni,fnj,probability,mu_d,sigma_dd,altratio] = link_hypothesis(TheConfig);
%function [fni,fnj,probability,mu_d,sigma_dd,altratio] = link_hypothesis(TheConfig);
%
% OUTPUT:
%     fni,fnj: feature indexes of hypothesized overlapping image pairs
%              note that fni < fnj always
% probability: probability of image overlap
%        mu_d: mean horizontal Euclidean distance between camera pair
%    sigma_dd: standard deviation
%    altratio: altitude ratio between camera pair based upon RDI, ideally should be 1
%  
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
%    12-21-2004      rme         Added altratio calculation, ideally want images to be similar
%                                in scale so that norm corr correspondence works
%    12-22-2004      rme         Added rounding of mu_d and altratio during sort to make sorting on
%                                altitude ratio more effective.
%    01-04-2005      rme         Cache altidue percentile calculation during creation of TheBathy data structure.
%    11-30-2006      rme         Updated function call from roundunit.m to roundb.m

global TheJournal TheBathy;

aclock('tic');
fprintf('==>%s: Computing candidate links... ',mfilename);

% define thresholds
minPercentOverlap = 0.35; % percent image overlap [0 1]
maxPercentOverlap = 0.95; % max is used to define a minimum baseline distance to image
			  % registration is more robust to small translation/rotation errors
minDistanceLowerLimit = 0.20; % [meters]
confidence = 0.80;        % confidence in overlap [0 1]


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

% data structure to keep track of image pair metrics
n = size(TheJournal.Links.plinks,1);
Distance.d        = spalloc(n,n,Nf);
Distance.std      = spalloc(n,n,Nf);
Distance.altratio = spalloc(n,n,Nf);

% compute Euclidean horizontal distance between camera centers
% and use it as a metric for hypothesizing overlapping image pairs
firstIteration = true;
fnj=Nf; % most recent image
for fni=Ni:Nf-1; % loop over all other images

    % extract pose information from our state estimate
    [x_lvi,x_lvj,x_vc,Sigma] = extract_poses(fni,fnj,TheConfig);
    
    % camera pose i in local-level frame
    [x_lci,Ji_plus] = head2tail(x_lvi,x_vc);

    % camera pose j in local-level frame
    if firstIteration;
      [x_lcj,Jj_plus] = head2tail(x_lvj,x_vc);
    end;
    
    % horizontal distance between camera centers
    [mu_d,J_ij] = euclideanDistance(x_lci(1:2),x_lcj(1:2));
    
    % total Jacobian w.r.t x_lvi, x_lvj, and x_vc via Chain-rule
    % i.e. d(mu_d)/d(x_lci(1:2),x_lcj(1:2)) * d(x_lci(1:2),x_lcj(1:2))/d(x_lvi,x_lvj,x_vc)
    J = J_ij * [Ji_plus(1:2,1:6), zeros(2,6),       Ji_plus(1:2,7:12);
		zeros(2,6),       Jj_plus(1:2,1:6), Jj_plus(1:2,7:12)];
    
    % 1st order covariance of horizontal distance
    Sigma_dd = J*Sigma*J';
    std_dd   = sqrt(Sigma_dd);
    
    % select 90th percentile altitude for camera rather than just
    % the max to be somewhat robust to altitude fliers
    alti = TheBathy(fni).alt90;
    altj = TheBathy(fnj).alt90;
    altmax   = max(alti,altj);
    altratio = alti/altj;
    
    % one dimensional image width
    imageWidth  = altmax*normWidth;
    maxDistance = (1-minPercentOverlap)*imageWidth;
    minDistance = (1-maxPercentOverlap)*imageWidth;
    minDistance = max(minDistance,minDistanceLowerLimit);
    
    % probability that the horizontal distance falls withing the range:
    % minDistance < d < maxDistance
    %if fni==24 || fni==600 || fni==22 || fni==23;
    %if fni==724 || fni==725 || fni==726 || fni==753 || fni==752 || fni==862;
    %if fni==659
      probability = normcdf(maxDistance, mu_d, std_dd) - normcdf(minDistance, mu_d, std_dd);
    %else;
    %  probability = 0;
    %end;
    if false%fni > 25
      x = linspace(-1,1,101)*6*std_dd + mu_d;
      x = x(:);
      y = 1/sqrt(2*pi*Sigma_dd)*exp(-(x-mu_d).^2 / (2*Sigma_dd));
      figure(105);
      plot(x,y,'g-');
      h = line([minDistance;minDistance],[0 max(y)]); set(h,'color','k');
      h = line([maxDistance;maxDistance],[0 max(y)]); set(h,'color','r');
      keyboard;
    end;
    
    % store computations
    Distance.d(fni,fnj) = mu_d;
    Distance.std(fni,fnj) = std_dd;
    Distance.altratio(fni,fnj) = altratio;
    TheJournal.Links.plinks(fni,fnj) = probability;

    %[probability,minDistance,mu_d,maxDistance,std_dd,altmax,altratio,fni,fnj]
    
end; % for fni

% crude 1D probability of horizontally overlapping image pairs
plinks = TheJournal.Links.plinks;

% candidate feature pairs which should have overlap with the given confidence
[fni,fnj] = find(plinks > confidence); % i < j always because testlinks is upper triangular
if isempty(fni);
  % set output arguments to empty
  [fni,fnj,probability,mu_d,sigma_dd,altratio] = deal([]);
  fprintf('none found\n');
  return;
end;
ii = sub2ind(size(plinks),fni,fnj);
probability = full(plinks(ii));
mu_d        = full(Distance.d(ii));
sigma_dd    = full(Distance.std(ii));
altratio    = full(Distance.altratio(ii));

% sort based upon probability, minimum distance, and then altitude ratio
% note: ideally we're looking for altitude ratios of 1 so that
% images are approximately the same scale and normalized correlation should work
[junk,ii] = sortrows([-roundb(probability,0.10), roundb(mu_d,0.5), roundb(abs(1-altratio),0.10)]);
fni = fni(ii);
fnj = fnj(ii);
mu_d = mu_d(ii);
sigma_dd = sigma_dd(ii);
probability = probability(ii);
altratio = altratio(ii);

% keep only the k most likey candidate pairs ensuring that we include the
% temporal pair if it's in the candidate set
k = min(length(fni), TheConfig.Estimator.max_link_hypoth);
itmp = find(fni==fnj-1);
if isempty(itmp) || itmp <= k;
  ii = 1:k;
else;
  ii = [itmp,1:k-1];
end;
fni = fni(ii);
fnj = fnj(ii);
mu_d = mu_d(ii);
sigma_dd = sigma_dd(ii);
probability = probability(ii);
altratio = altratio(ii);

fprintf('done %.2fs',aclock('toc'));

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
