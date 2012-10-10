function [Pbb_est,omega] = covfusion(Paa,Pab,PPbb,minimize);
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

Options = optimset;
Options.Display = 'on';
Options.LargeScale = 'off';
Options.LevenberMarquardt = 'on';
Options.MaxFunEvals = 1e7;
omega = fmincon(fhandle,0.9,[],[],[],[],0,1,@nonlincon,Options,Paa,Pab,PPbb);

Pbb_est = omega*PPbb;

%==========================================================
function tr = mytrace(omega,Paa,Pab,PPbb);
tr = trace([Paa Pab; Pab' omega*PPbb]);
%===========================================================
function dt = mydet(omega,Paa,Pab,PPbb);
dt = det([Paa Pab; Pab' omega*PPbb]);

%===========================================================
function [C,Ceq] = nonlincon(omega,Paa,Pab,PPbb);
Nb = length(PPbb);
Ceq = [];
for n=1:Nb
  C(n,1) = -det([Paa,         Pab(:,1:n); ...
		 Pab(:,1:n)', omega*PPbb(1:n,1:n)]);
end;
