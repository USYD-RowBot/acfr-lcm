function smatrix = zernikeCorrScorePolar(A1_nm,A2_nm,Zbasis);
%Zernike Correlation Score Polar Coordinates
%    smatrix = zernikeCorrScorePolar(A1_nm,A2_nm,Zbasis) returns a [M x N]
%    matrix containing all correlation scores between patches encoded in A1_nm
%    and patches encoded in A2_nm.  Note, A1_nm and A2_nm are matrices of complex
%    normalized Zernike moments of size [len x M] and [len x N] respectively.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-15-2004      rme         Created and written.

% note: assumes A1_nm & A2_nm are *normalized* Zernike moments and that
% no additional weighting is required in order to compute correlation score.

% account for missing negative repeitions -m
% note: A_nm only stores positive repetitions m, since A_n-m = A_nm* 
% where * denotes complex conjugate.
m = Zbasis.zindex(2,:)';
ii = find(m ~= 0);
A1_nm = [A1_nm; conj(A1_nm(ii,:))];
A2_nm = [A2_nm; conj(A2_nm(ii,:))];

% compute all correlation scores
% note: by definition the transpose operator conjugates A1_nm
smatrix = real(A1_nm'*A2_nm);
