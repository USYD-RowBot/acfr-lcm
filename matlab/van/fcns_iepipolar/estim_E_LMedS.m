function [e_min,inlier_sel,iterations] = estim_E_LMedS(x1,y1,x2,y2,percent_inliers)
%ESTIM_E_LMEDS  Robust estimation of essential matrix using
%               Least Median of Squares (LMedS)
%   [e_min,sel,iterations] = ESTIM_E_LMEDS(x1,y1,x2,y2,percent_inliers)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09/15/2003      rme         Created.
%    09/24/2003      rme         Added check at to see if a consistent
%                                solution was found.
%    09/27/2003      rme         Added regularized random sampling via
%                                initbucket.m and randbucket.m
%    2004-12-21      rme         Modified calculation of confidence level alpha

%=============================
% SET PARAMETERS
%=============================
MIN_ITERATIONS = 50;
MAX_ITERATIONS = 500;
alpha = 1-2*normcdf(-4);  % confidence level of picking an inlier set
p = length(x1);           % set size
q = 6;                    % sample size (for 6-point algorithm)
if ~exist('percent_inliers','var')
  percent_inliers = 0.5;
end

% assuming percent_inliers, precompute number of trials to perform to achieve
% desired confidence level
N = adapt_trials(percent_inliers*p,p,alpha,q);
N = max(N,MIN_ITERATIONS);
N = min([N,MAX_ITERATIONS]);

% initialize regularly random selection method using [8 x 8] grid of buckets
b_t = initbucket(x1,y1,8);

%==============================
% LMedS
%==============================
D = F_meas_matrix(x1,y1,x2,y2);

min_median_error = inf;
iterations = 0;
while iterations < N
  % select a random sample of "q" putative correspondences
  if q < size(b_t.bin,1)
    rsel = randbucket(b_t,q);
  else
    rsel = randsample(p,q);
  end
  
  % extract the corresponding sample measurement matrix
  Dsamp = D(rsel,:);
  
  % estimate the Essential matrix using Oscar's 6-point algorithm
  % (2, 4, or 6 solutions)
  [estruc_samp,n_e] = estim_E_6p(Dsamp);
  
  % for each of the solutions calculate the Sampson distance
  % and determine number of inliers
  for k = 1:n_e
    [dsampson,csampson] = sampson_distance(estruc_samp(k).e,D,x1,y1,x2,y2);
    med_error = median(dsampson);
    
    % check if current solution has a lower median error than the previous best solution
    if med_error < min_median_error
      min_median_error = med_error;
      e_min = estruc_samp(k).e;      
    end
  end
  iterations = iterations+1;
end

if min_median_error ~= inf
  % the *robust standard deviation* estimate is given by the following
  % equation taken from Faugeras' GMI pg. 334
  sigma_robust = 1.4826*[1 + 5/(p-q)]*sqrt(min_median_error);

  % based on sigma_robust, we can select an inlier set according to
  % inlier:  if residual^2 <= (2.5*sigma_robust)^2
  % outlier: otherwise
  % This again comes from Faugers GMI pg. 334
  [dsampson,csampson] = sampson_distance(e_min,D,x1,y1,x2,y2);
  inlier_sel = find(dsampson <= (2.5*sigma_robust)^2);
else
  % no consistent set of correspondences and model could be found
  e_min = [];
  inlier_sel = [];
end
