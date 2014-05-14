function [varargout] = reprojection_error(varargin);
%REPROJECTION_ERROR calculates the optimal measuremetns and reprojection error.
%  INPUT:
%  1) F,U1_meas,U2_meas
%  2) F,u1_meas,v1_meas,u2_meas,v2_meas
%  OUTPUT:
%  1) e1,e2 the componentwise reprojection error as measured in image
%     space for each set of image points. i.e.
%     e1 = [du1_1, dv1_1, ... ,du1_n, dv1_n]'
%     e2 = [du2_1, dv2_1, ... ,du2_n, dv2_n]'
%  2) e1,e2,U1_ideal,U2_ideal reprojection error (Euclidean distance) 
%     as measured in image space i.e.
%     e1 = [d([u1_meas,v1_meas],[u1_ideal,v1_ideal]) ...]'
%     e2 = [d([u2_meas,v2_meas],[u2_ideal,v2_ideal]) ...]'  
%  3) e1,e2,u1_ideal,v1_ideal,u2_ideal,v2_ideal
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2003            op          Created and written.
%    09/20/2003      rme         Cleaned up implementation.

%====================================
% CHECK INPUT ARGUMENTS
%====================================
switch nargin
 case 3 
  F = varargin{1};
  U1_meas = varargin{2}; % [3xN] homogenous point matrices
  U2_meas = varargin{3};
 case 5 
  F = varargin{1};
  u1_meas = varargin{2}; % [Nx1] point vectors
  v1_meas = varargin{3};
  u2_meas = varargin{4};
  v2_meas = varargin{5};
  U1_meas = homogenize(u1_meas,v1_meas);
  U2_meas = homogenize(u2_meas,v2_meas);
 otherwise
  error('Incorrect number of arguments');
end

% initialize ideal points to measured points
U1_ideal = U1_meas;
U2_ideal = U2_meas;

% iterations that refine ML estimate of image points by implicit
% triangulation.  this implementation follows Kanatani's notation:
% x = x - W/V*E
for k = 1:10  
  F1 = F *U1_ideal;
  F2 = F'*U2_ideal;
  
  W = sum(U2_ideal.*F1);
  V = sum(U2_ideal.*(F*F2) + U1_ideal.*(F'*F1));

  G = W./V;
  Gmat = repmat(G,[3 1]);
  U1_ideal = U1_ideal - Gmat.*F2;
  U2_ideal = U2_ideal - Gmat.*F1;
end

% dehomogenize
U1_ideal = U1_ideal./repmat(U1_ideal(3,:),[3 1]);
U2_ideal = U2_ideal./repmat(U2_ideal(3,:),[3 1]);


%======================================
% CHECK OUPUT ARGUMENTS
%======================================
switch nargout
 case 2
  % output e1 and e2 as components of distance
  % used in ML estimation of pose
  e1 = U1_meas(1:2,:)-U1_ideal(1:2,:);
  e1 = e1(:); % stack into column vector e1 = [dx1_1, dy1_1, ... ,dx1_n, dy1_n]'
  e2 = U2_meas(1:2,:)-U2_ideal(1:2,:);
  e2 = e2(:); % stack into column vector e2 = [dx2_1, dy2_1, ... ,dx2_n, dy2_n]'
  varargout{1} = e1;
  varargout{2} = e2;
 case 4
  % output e1 and e2 as distances
  % used in RANSAC estimation of E
  e1 = sum((U1_meas(1:2,:)-U1_ideal(1:2,:)).^2);
  e2 = sum((U2_meas(1:2,:)-U2_ideal(1:2,:)).^2);
  varargout{1} = e1';
  varargout{2} = e2';
  varargout{3} = U1_ideal;
  varargout{4} = U2_ideal;
 case 6
  % output e1 and e2 as distances
  % used in RANSAC estimation of E
  e1 = sum((U1_meas(1:2,:)-U1_ideal(1:2,:)).^2);
  e2 = sum((U2_meas(1:2,:)-U2_ideal(1:2,:)).^2);  
  [u1_ideal,v1_ideal] = dehomogenize(U1_ideal);
  [u2_ideal,v2_ideal] = dehomogenize(U2_ideal);
  varargout{1} = e1';
  varargout{2} = e2';
  varargout{3} = u1_ideal;
  varargout{4} = v1_ideal;
  varargout{5} = u2_ideal;
  varargout{6} = v2_ideal;
 otherwise
  error('Incorrect number of output arguments');
end
