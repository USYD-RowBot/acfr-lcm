function varargout = cook_xyz(ts,x_rs,y_rs,z_rs,X_vs,X_lr,tv,r_lv,p_lv,h_lv);
%function varargout = cook_xyz(ts,x_rs,y_rs,z_rs,X_vs,X_lr,tv,r_lv,p_lv,h_lv)
% ts   = n-vector of sensor measurement timebase
% x_rs,y_rs,z_rs = n-vector of sensor position w.r.t. sensor's reference frame,
%        i.e., X_rs(1:3). if an element is not measured, then denote it by an
%        empty matrix [].
% X_vs = 6-vector, sensor frame w.r.t. vehicle frame
% X_lr = 6-vector, sensor's reference frame w.r.t. local-level
% tv   = m-vector of vehicle attitude timebase
% r_lv,p_lv,h_lv (ea.) = m-vector of vehicle attitude w.r.t. local level, 
%        i.e., X_lv(4:6).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who              What
%    -----------     ------------     ----------------------------
%    04-30-2006      Ryan Eustice     Created and written.

n = length(ts);

% interpolate vehicle attitude to sensor timebase
rphi = interp1(tv,[r_lv(:),p_lv(:),h_lv(:)],ts);
R_lv = rotxyz(rphi(:,1),rphi(:,2),rphi(:,3));

% sensor orientation w.r.t. local-level
R_vs = rotxyz([X_vs(4),X_vs(5),X_vs(6)]);
R_vs = repmat(R_vs,[1 1 n]); % stack along 3rd dim
R_ls = multiprod(R_lv,R_vs);

% sensor orientation w.r.t. sensor's reference frame
R_lr = rotxyz([X_lr(4),X_lr(5),X_lr(6)]);
R_rl = R_lr';
R_rl = repmat(R_rl,[1 1 n]); % stack along 3rd dim
R_rs = multiprod(R_rl,R_ls);
[r_rs,p_rs,h_rs] = rot2rph(R_rs);

% sensor pose w.r.t. sensor's reference frame
a = ones(3,1);
if isempty(x_rs);
  a(1) = 0;
  x_rs = zeros(n,1);
end;
if isempty(y_rs);
  a(2) = 0;
  y_rs = zeros(n,1);
end;
if isempty(z_rs);
  a(3) = 0;
  z_rs = zeros(n,1);
end;
X_rs = [x_rs(:), y_rs(:), z_rs(:), r_rs(:), p_rs(:), h_rs(:)]';

% vehicle pose w.r.t. local-level
X_rv = ssc_head2tail(X_rs,ssc_inverse(X_vs));
X_lv = ssc_head2tail(X_lr,X_rv);


% output cooked elements
ii = 1;
if (a(1) > 0);
  varargout{ii} = X_lv(1,:)';
  ii = ii+1;
end;
if (a(2) > 0);
  varargout{ii} = X_lv(2,:)';
  ii = ii+1;
end;
if (a(3) > 0);
  varargout{ii} = X_lv(3,:)';
end;
