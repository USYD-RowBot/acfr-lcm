function Nmax = adapt_trials(n_inlier,n_total,p,s)
%ADAPT_TRIALS  Number of trials required to achieve an outlier-free sample set.
%  N = ADAPT_TRIALS(n_inlier,n_total,p,s) calculates number of
%  trials needed to get a outlier-free sample set of size s with
%  probability p from a set of n_total measurements.
%
%  Reference: Algorithm 3.5 Hartley/Zisserman
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-2002         op          Created and written.
%    12/20/2002      rme         Renamed to adapt_trials.m and
%                                commented code.

% fraction of outliers
epsilon = 1 - n_inlier/n_total;

% fraction of inliers
% (1-epsilon)

% probability of drawing s inliers
% (1-epsilon)^s

% probability of drawing a set with one or more outliers
% 1-(1-epsilon)^s

% probability of drawing a set with one or more outliers N times
% (1-(1-epsilon)^s)^N

% probability of drawing at least one set of s inliers
% p = 1-(1-(1-epsilon)^s)^N

% calculate number of trial required to achieve desired confidence level
% (1-(1-epsilon)^s)^N = 1-p
Nmax = log10(1-p)/(log10(1-(1-epsilon)^s)+eps);
