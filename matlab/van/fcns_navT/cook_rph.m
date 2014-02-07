function [r_lv,p_lv,h_lv] = cook_rph(r_rs,p_rs,h_rs,X_vs,X_lr);
%function [r_lv,p_lv,h_lv] = cook_rph(r_rs,p_rs,h_rs,X_vs,X_lr);
%-----------------------------------------------------------------
%    History:
%    Date            Who              What
%    -----------     ------------     ----------------------------
%    04-30-2006      Ryan Eustice     Created and written.

n = length(r_rs);

% sensor frame w.r.t. reference frame
R_rs = rotxyz(r_rs,p_rs,h_rs);

% reference frame w.r.t. local-level
R_lr = rotxyz(X_lr(4),X_lr(5),X_lr(6));
R_lr = repmat(R_lr,[1 1 n]); % stack along 3rd dim

% sensor frame w.r.t. vehicle frame
R_vs = rotxyz(X_vs(4),X_vs(5),X_vs(6));

% vehicle frame w.r.t. sensor frame
R_sv = repmat(R_vs',[1 1 n]); % stack along 3rd dim

% vehicle frame w.r.t. local-level
R_lv = multiprod(R_lr, multiprod(R_rs, R_sv));
[r_lv,p_lv,h_lv] = rot2rph(R_lv);

% unwrap angles
r_lv = funwrap(r_lv);
p_lv = funwrap(p_lv);
h_lv = funwrap(h_lv);
