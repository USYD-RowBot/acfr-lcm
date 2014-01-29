function [J2,J] = sampson_jacobian(f,x1,y1,x2,y2)
%SAMPSON_JACOBIAN  Compute Jacobian matrix w.r.t. coordinates of xj'*F*xi
%   [J2,J] = SAMPSON_JACOBIAN(f,x1,y1,x2,y2) returns J2, the sum of
%   squares of the jacobian elements, and J, which is a matrix with
%   the Jacobian for each correspondence as a row.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-2002         op          Created and written.
%    12/19/2002      rme         Modified to work with normalized coordinates.

F = reshape(f,3,3)';

J1 = [x1, y1, ones(size(x1))]*F';
J2 = [x2, y2, ones(size(x2))]*F;

J  = [J1(:,1) J1(:,2) J2(:,1) J2(:,2)];
J2 = J1(:,1).^2 + J1(:,2).^2 + J2(:,1).^2 + J2(:,2).^2;
