function Ipolar = zernikeReconstructPolar(A_nm,Zbasis);
%Zernike Reconstruct Polar Represenation
%    Ipolar = zernikeReconstructPolar(A_nm,Zbasis) approximately reconstructs
%    the portion of image I within the unit circle using the computed Zernike
%    moments A_nm.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-15-2004      rme         Created and written.

% make A_nm a column vector if it isn't already
if size(A_nm,2) > 1;
  % note: have to be careful here, since A_nm is complex we can't just use
  % the transpose operator because it will conjugate the result
  A_nm = reshape(A_nm,[],1);
end;

% A_nm and V_nm only contain positive repetitions m, account for the
% missing -m terms in the reconstruction
% note: A_n,-m = A_nm* where * denotes complex conjugate
m  = Zbasis.zindex(2,:);
ii = find(m ~= 0);
A_nm(ii) = 2*A_nm(ii); % account for effect of complex conjugate

% reconstruct polar image patch from Zernike moments by summing over all A_nm*V_nm
Ipolar = real(Zbasis.V_nm * A_nm);        % vectorized polar image patch
Ipolar = reshape(Ipolar,Zbasis.sampSize); % reshape into [tsamp,rsamp] polar array
