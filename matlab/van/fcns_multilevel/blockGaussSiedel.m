function x = blockGaussSiedel(A,b,x,Nb,Ncycles);
%function x = blockGaussSiedel(A,b,x,Nb,Ncycles);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-20-2004      rme         Created and written.

Nblocks = length(b)/Nb;
for i=1:Ncycles;
  % 1 iteration of Gauss-Siedel
  ii = 1:Nb;
  for j=1:Nblocks;
    x(ii) = x(ii) + A(ii,ii) \ (b(ii) - A(ii,:)*x);
    ii = ii+Nb;
  end;
end;
