function [dsampson,csampson] = sampson_distance(f,D,x1,y1,x2,y2)
%SAMPSON_DISTANCE  Calculates the Sampson distances for a set of correspondences.
%   [DSAMPSON,CSAMPSON] = SAMPSON_DISTANCE(f,D,x1,y1,x2,y2) returns
%   the vector DSAMPSON of Sampson distances for the
%   correspondences and the scalar total cost CSAMPSON.  f is the
%   [9x1] fundamental matrix parameters, D the [Nx9] measurement
%   matrix, and x1, y1, x2, y2 are [Nx1] vectors.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-2002         op          Created and written.
%    12/19/2002      rme         Modified to work with normalized coordinates.

[J2,J] = sampson_jacobian(f,x1,y1,x2,y2);
dsampson = ((D*f).^2)./J2;
csampson = sum(dsampson); %total cost

