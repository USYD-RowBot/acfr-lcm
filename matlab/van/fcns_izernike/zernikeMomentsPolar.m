function A_nm = zernikeMomentsPolar(fv,Zbasis,weightFcn);
%Zernike Moments Polar Image Representation
%    A_nm = zernikeMomentsPolar(fv,Zbasis,weightFcn) computes the Zernike moments
%    for a matrix of vectorized polar image patches fv.
%
%          fv: [len x Nf] matrix of vectorized polar image patches
%      Zbasis: Zernike basis functions structure
%   weightFcn: 'regular' computes the standard Zernike moments
%              'xcorr'   computes the normalized Zernike moments useful
%                        for calculating normalized cross-correlation similarity score
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-15-2004      rme         Created and written.

n  = Zbasis.zindex(1,:);  % Zernike order
 
switch lower(weightFcn);
case 'regular';
 % weight the basis functions so they yield the Zernike moments
 V_nm = ( Zbasis.darea(:) * (n+1)/pi ) .* Zbasis.V_nm;
case 'xcorr';
 % weight the basis functions so they yield the normalized Zernike moments
 V_nm = ( Zbasis.darea(:) * sqrt((n+1)/pi) ) .* Zbasis.V_nm; 
otherwise;
 error('unknown weightFcn');
end;

% project the sampled pixels within the unit disk onto the Zernike basis
% functions to get the associated Zernike moments for all image patches.
% note: by definition the transpose operator conjugates V_nm
A_nm = V_nm'*fv;
