function [X,t,mindx,dh] = runsim(goal_t,samp_period,X_o)
%RUNSIM  run a trajectory simulation.
%  [X,T,MINDX] = RUNSIM(GOAL_T,SAMP_PERIOD,X_O) X_O is initial condition
%  and is an optional argument.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-20-2003      rme         Created and written.

t = 0;
if ~exist('X_o','var')
  X = zeros(12,1);
  X(3) = -25; % assign a depth
else
  X = X_o;
end
mindx = 0;
dh = NaN;
for ii=1:length(goal_t)
  [X_tmp,t_tmp,mindx_tmp,dh_tmp] = simtraj(t(end),X(:,end),goal_t(ii),samp_period);
  t = [t, t_tmp];
  X = [X, X_tmp];
  mindx = [mindx,mindx_tmp];
  dh = [dh,dh_tmp];
end
