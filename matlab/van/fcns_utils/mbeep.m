function mbeep(n,dt)
%MBEEP multiple system beeps.
%   MBEEP(N) beeps N times.
%
%   MBEEP(N,DT) beeps N times with a pause of length DT in between successive beeps.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-02-2003      rme         Created and written.

if ~exist('n','var') || isempty(n); n=2; end;
if ~exist('dt','var') || isempty(dt); dt=0.2; end;

for ii=1:n;
  beep;
  if n > 1;
    pause(dt);
  end;
end;
