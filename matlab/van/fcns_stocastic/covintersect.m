function [Pcc,omega] = covintersect(Paa,Pbb,minimize);
%COVINTERSECT   Covariance Intersection
%  [Pcc,omega] = covintersect(Paa,Pbb,MINIMIZE) computes a consistent
%  fused covariance Pcc from the covariances Paa and Pbb based upon
%  the principle of Covariance Intersection.
%
%  Pcc^-1 = omega*Paa^-1 + (1-omega)*Pbb^-1
%
%  Pcc is computed based upon the optimize criteria specified by the string
%  argument MINIMIZE which can be {'trace','det'}.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-20-2004      rme         Created and written.

switch lower(minimize);
case 'trace'; fhandle = @mytrace;
case 'det';   fhandle = @mydet;
otherwise; error(sprintf('unkown option %s',minimize));
end;

invPaa = spdinverse(Paa);
invPbb = spdinverse(Pbb);

Options = optimset;
Options.Display     = 'off';
Options.LargeScale  = 'off';
Options.MaxFunEvals = 1e6;
OPtions.MaxIter     = 1e2;
Options.TolFun      = eps;
Options.TolX        = eps;
omega = fmincon(fhandle,0.5,[],[],[],[],0,1,[],Options,invPaa,invPbb);

invPcc = omega*invPaa + (1-omega)*invPbb;
Pcc = spdinverse(invPcc);

%==========================================================
function tr = mytrace(omega,invPaa,invPbb);
invPcc = omega*invPaa + (1-omega)*invPbb;
tr = trace(spdinverse(invPcc));

%===========================================================
function dt = mydet(omega,invPaa,invPbb);
invPcc = omega*invPaa + (1-omega)*invPbb;
dt = det(invPcc)^(-1/length(invPcc)); % normalized determinant of Pcc
