function [e_min,inlier_sel,iterations] = estim_E_RANSAC(x1,y1,x2,y2,percent_inliers)
%ESTIM_E_RANSAC Robust estimation of essential matrix using
%               RANdom SAMple Consensus (RANSAC)
%   [e_min,sel,iterations] = ESTIM_E_RANSAC(x1,y1,x2,y2,percent_inliers)
%
%   note: percent_inliers is unspecified defaults to 0.5;
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09/15/2003      rme         Created.
%    09/24/2003      rme         Added check at to see if a consistent
%                                solution was found.
%    09/27/2003      rme         Added regularized random sampling via
%                                initbucket.m and randbucket.m
%    04-14-2004      rme         Created from estim_E_LMedS.m

%=============================
% SET PARAMETERS
%=============================
THRESH = 1e-6;
MIN_ITERATIONS = 50;
MAX_ITERATIONS = 500;
alpha = 0.999;  % confidence level of picking an inlier set
p = length(x1); % set size
q = 6;          % sample size (for 6-point algorithm)
if ~exist('percent_inliers','var')
  percent_inliers = 0.5;
end

% assuming percent_inliers, precompute number of trials to perform to achieve
% desired confidence level
N = adapt_trials(percent_inliers*p,p,alpha,q);
N = min([N,MAX_ITERATIONS]);

% initialize regularly random selection method using [8 x 8] grid of buckets
b_t = initbucket(x1,y1,8);

%==============================
% RANSAC
%==============================
D = F_meas_matrix(x1,y1,x2,y2);

e_min  = [];
inlier_sel = [];
n_in_e = 0;
iterations = 0;
while (iterations < MIN_ITERATIONS) || (iterations < N)
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
    sel_in = (dsampson < THRESH);
    n_in(k) = sum(sel_in);
    
    % check if current solution has more consensus than 
    % previous best solution
    if n_in(k) > n_in_e
      n_in_e = n_in(k);
      sel_in_e = sel_in;
      e_min = estruc_samp(k).e;
      
      % adaptively calculate number of iterations to perform based upon
      % current estimate of inlier to outlier ratio
      percent_inliers = n_in_e/p;
      N = adapt_trials(percent_inliers*p,p,alpha,q);
      N = min(N,MAX_ITERATIONS);
    end
  end
  iterations = iterations+1;
end

inlier_sel = find(sel_in_e);
