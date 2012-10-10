function [rr_v,pr_v,hr_v] = cook_rphdot(rr_s,pr_s,hr_s,X_vs);
%function [rr_v,pr_v,hr_v] = cook_rphdot(rr_s,pr_s,hr_s,X_vs);
%-----------------------------------------------------------------
%    History:
%    Date            Who              What
%    -----------     ------------     ----------------------------
%    04-30-2006      Ryan Eustice     Created and written.

% sensor frame w.r.t. vehicle frame
R_vs = rotxyz(X_vs(4),X_vs(5),X_vs(6));

% angular rates in vehicle frame
ret = R_vs * [rr_s(:), pr_s(:), hr_s(:)]';
rr_v = ret(1,:)';
pr_v = ret(2,:)';
hr_v = ret(3,:)';
