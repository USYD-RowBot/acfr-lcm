function polarpatch = zernikeDetrendPolar(polarpatch,Zbasis);
%Zernike Polar Image Patch Detrending
%    polarpatch = zernikeDetrendPolar(polarpatch,Zbasis) normalizes the polar
%    coordinate image patch by demeaning and  diving by the patch energy.  This
%    normalization yields a normalized correlation score in the range [-1,1].
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-16-2004      rme         Created and written.

% note pixels are not equally weighted area-wise in a polar representation,
% therefore take this into consideration when calculating mean and energy.
polarmean   = polarpatch(:)' * Zbasis.darea(:)/pi;  % total area = pi*r^2 = pi for unit circle
polarpatch  = polarpatch - polarmean;               % demean
polarenergy = polarpatch(:)' * (polarpatch(:) .* Zbasis.darea(:));
polarpatch  = polarpatch / sqrt(polarenergy);       % normalize
