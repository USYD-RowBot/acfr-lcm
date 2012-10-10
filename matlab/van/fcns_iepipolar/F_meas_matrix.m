function D = F_meas_matrix(xi,yi,xj,yj)
%F_MEAS_MATRIX  Returns the measurement matrix for SVD calculation of f.
%  D = F_MEAS_MATRIX(xi,yi,xj,yj) computes the [Nx9] measurement
%  matrix D used for SVD calculation of the fundamental matrix f
%  represented as a [9x1] vector.  xi, yi, xj, yj are [Nx1] vectors
%  of point correspondences where xj'*F*xi = 0.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-2002         op          Created and written.
%    12/20/2002      rme         Modified to work with normalized coordinates.


n = length(xi);
D = [xj.*xi, xj.*yi, xj, yj.*xi, yj.*yi, yj, xi, yi, ones(n,1)];

