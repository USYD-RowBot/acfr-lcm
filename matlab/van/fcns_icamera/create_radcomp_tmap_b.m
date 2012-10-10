function tmap_b = create_radcomp_tmap_b(nr,nc,tf_add_distortion)
%function tmap_b = create_radcomp_tmap_b(nr,nc,tf_add_distortion)
%
% overall description of function:  
% precompute a look-up-table of distorted pixel coordinates to use with
% tformarray to warp raw imagery compensating for distortion.
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-28-2003      rme         Created and written.
%    12-17-2004      rme         Updated [u,v] calculation.

% compensated image pixel grid
[u,v] = deal( repmat([0:nc-1],[nr 1]), repmat([0:nr-1]',[1 nc]) );

% map compensated grid to distorted grid
u_d = tformfwd([u(:),v(:)],tf_add_distortion);
clear u v;
[u_d,v_d] = deal(u_d(:,1),u_d(:,2));
u_d = reshape(u_d,[nr nc]);
v_d = reshape(v_d,[nr nc]);

% note that when using tformarray, i'm adding 1 to the (u_d,v_d)
% coordinates to be consistent with matlab array indexing since i
% define the top left pixel to be (0,0) while matlab defines it to be (1,1)
tmap_b = cat(3,u_d+1,v_d+1);
