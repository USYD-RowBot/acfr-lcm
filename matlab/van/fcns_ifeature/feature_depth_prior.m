function [Z_prior,Cov_Z_prior] = feature_depth_prior(ui,vi,ub,vb,Z_bathy,Cov_Z_bathy,TOL)
%function [Z_prior,Cov_Z_prior] = feature_depth_prior(ui,vi,ub,vb,Z_bathy,Cov_Z_bathy,TOL)  
%   INPUT
%   [ui,vi] - feature point pixel coordinates
%   [ub,vb] - bathy in pixel coordinates
%   Z_bathy - bathy measured scene depth in camera coordinates
%   Cov_Z_bathy - currently NOT USED
%   TOL     -  max euclidean pixel distance
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-01-2004      rme         Created and written.
%    04-05-2004      rme         commented out Cov_Z_bathy because
%                                ellipse for Cov_Z_bathy >> ellipse for Zmax_error
%    04-15-2004      rme         Changed the Z_prior to use the median
%                                instead of the average scene depth
%    03-15-2005      rme         Changed Zmax_error to use prctile instead of max

m = length(ui);
n = length(ub);  

% median scene depth according to bathy
Zmed = median(Z_bathy);

% worst case bound on covariance according to bathy using min/max bounds
%Zmax_error = max(abs(Z_bathy - Zmed));
Zmax_error = prctile(abs(Z_bathy - Zmed),75);
Cov_Zmed = Zmax_error^2;

% initialize scene depth prior for all points to Zmed & Cov_Zmed
Z_prior = repmat(Zmed,[m 1]);
Cov_Z_prior = repmat(Cov_Zmed,[m 1]);
Cov_Z_prior = repmat(Cov_Zmed,[m 1]);

% now look for feature points within TOL distance from bathy pixels and give
% them a more accurate scene depth prior based upon bathy.  note that i'm
% taking advantage of matlab's vector math capabilities to do this
% computation w/o a "for loop"
%
% euclidean distance between feature pixel coordinates
% and bathy pixel coordinates
% d^2 = [(ui-ub)^2 + (vi-vb)^2] < TOL^2
d2 = (repmat(ui,[1 n])-repmat(ub',[m 1])).^2  + ...
     (repmat(vi,[1 n])-repmat(vb',[m 1])).^2;

% find closest bathy point to feature point
[d2_min,bind] = min(d2,[],2);

% find points within TOL radius
iind = find(d2_min < TOL^2);

if length(iind) > 0 % update prior
  Z_prior(iind) = Z_bathy(bind(iind));
  %Cov_Z_prior(iind) = Cov_Z_bathy(bind(iind));
end
