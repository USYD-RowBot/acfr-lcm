function Ic = undistort_image(Iraw,tmap_b)
%function Ic = undistort_image(Iraw,tmap_b)  
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-28-2003      rme         Created and written.
  
R = makeresampler('linear','fill');
Ic = tformarray(Iraw,[],R,[2 1],[1 2],[],tmap_b,0);
