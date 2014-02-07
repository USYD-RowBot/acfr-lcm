function Zbasis = zernikeBasisFcnsPolar(sampSize,order);
%Zernike Basis Functions in a Polar Representation
%    Zbasis = zernikeBasisFcnsPolar(sampSize,order) computes a data structure
%    containing all Zernike polynomial basis functions up the specified order
%    evaluated over an image patch of size patchSize.
%
%    The data structure Zbasis contains the following fields:
%     sampSize: row,column size of image patch in polar coordinates for which
%               basis has been calculated
%  xsamp,ysamp: nonuniform Cartesian coordinate image sample points      
%  rsamp,tsamp: uniform Polar coordinate image sample points
%         size: [len, repetitions]
%       zindex: [2 x ...] matrix organized by [order; repetition] i.e. [n;m]
%         V_nm: [len x ...] matrix of Zernike polynomials evaluated at pixels rsamp,tsamp
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-15-2004      rme         Created and written.

% sample space in polar coordinates
%      0       1
%     o----------->
%   0 | 
%     | array
%     |
% 2pi v
%  
dr = 1/sampSize(2);
dt = 2*pi/sampSize(1);
[rsamp,tsamp] = meshgrid( dr/2:dr:1-dr/2, 0:dt:2*pi-dt );  % uniform rho spacing
% note: the Zernike radial magnitude function R(rho) disproportionately weights
%       small values of rho and emphasizes values near 1.  therefore, the
%       radial samples for small rho have little effect in the Zernike integral.
%       to get a more accurate estimate of the integral, rather than decreasing the
%       discretization spacing of rho (which results in increased number of computations)
%       instead use an exponential spacing of sample points for rho so that more samples
%       occur as rho approaches 1.
%rr = 1-exp(-linspace(0,2,sampSize(2)));                     % exponential rho spacing
%rr = rr/max(rr);  % range [0,1]
%[rr(1),rr(end)] = deal(dr/2, 1-dr/2);
%[rsamp,tsamp] = meshgrid( rr, 0:dt:2*pi-dt );               % uniform theta spacing

% calculate area differential in polar coordinates
darea = rsamp*dr*dt;
%delta = repmat(dt,sampSize(1),1) * [dr/2, diff(rsamp(1,:))];
%darea = rsamp .* delta;

% nonuniform cartesian sample space
[xsamp,ysamp] = deal(rsamp.*cos(tsamp), rsamp.*sin(tsamp));

% calculate the total number of repetitions
repetitions = 0;
for n=0:order;
  m = zernikeRepetitions(n);
  repetitions = repetitions + length(m);
end;

% evaluate the Zernike polynomial for all orders and repetitions
rho = rsamp(:);
theta = tsamp(:);
len = length(rho);
V_nm = zeros(len,repetitions);
zindex = zeros(2,repetitions);
ii = 1;
for n=0:order;
  for m=zernikeRepetitions(n);
    V_nm(:,ii) = zernikePolynomial(n,m,rho,theta);    
    zindex(:,ii) = [n;m];
    ii = ii+1;
  end;
end;

% stuff output data strucure
Zbasis.order    = order;
Zbasis.sampSize = sampSize;
Zbasis.xsamp    = xsamp;
Zbasis.ysamp    = ysamp;
Zbasis.rsamp    = rsamp;
Zbasis.tsamp    = tsamp;
Zbasis.darea    = darea;
Zbasis.size     = [len, repetitions];
Zbasis.zindex   = zindex;
Zbasis.V_nm     = V_nm;
