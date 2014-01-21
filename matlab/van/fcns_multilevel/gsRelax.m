function x = gsRelax(A,x,b,N);
%function x = gsRelax(A,x,b,N);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-12-2005      rme         Created and written.

L = tril(A);
for n=1:N;
  % 1 vectorized iteration of Gauss-Siedel
  x = x + L \ (b - A*x);
end;
