function smatrix = zernikeCorrScorePolarPatch(patchmat1,patchmat2,Zbasis);
%Zernike Polar Image Patch Correlation Score
%    smatrix = zernikeCorrScorePolarPatch(patchmat1,patchmat2,Zbasis) computes
%    normalized correlation score between the two sets of vectorized polar
%    image patches assuming that each patch has already been detrended via
%    zernikeDetrendPolar.m.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-16-2004      rme         Created and written.

% note pixels are not equally weighted area-wise in a polar representation,
% therefore take this into consideration when calculating correlation score.
smatrix = patchmat1' * ( patchmat2 .* repmat(Zbasis.darea(:),[1 size(patchmat2,2)]) );
