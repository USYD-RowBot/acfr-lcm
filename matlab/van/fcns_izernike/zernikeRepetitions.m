function m = zernikeRepetitions(n)
%Zernike Repetitions
%    m = zernikeRepetitions(n) calculates valid positive repetition indices, m,
%    for Zernike moments of order n subject to the following constraints:
%    abs(m)<=n  AND  n-abs(m) is even
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-09-2004      rme         Created and written.

% n odd,  abs(m) = 1:2:n;
% n even, abs(m) = 0:2:n;
m = rem(n,2):2:n;
