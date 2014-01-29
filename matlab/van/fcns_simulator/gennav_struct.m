function nav_t = gennav_struct(X,t,mindx,slist)
%GENNAV_STRUCT generates a nav_t structure from simulation results.
%   NAV_T = GENNAV_STRUCT(X,T,MINDX,SLIST) creates a NAV_T structure
%   using the [12 x N] sampled state vector X, sample times [1 x N] T, 
%   [1 x N] sensor index MINDX, and cell string array SLIST.
%
%   Example usage:
%   nav_t = gennav_struct(X,t,mindx,{'RDI','PARO','XBOW','PXF'});
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-21-2003      rme         Created and written.
%    12-19-2003      rme         Added XBOW, PARO, PXF

n = length(t);
t = reshape(t,[1 n]);
X = reshape(X,[12 n]);
mindx = reshape(mindx,[1 n]);

% define shorthand index for state vector elements
% vehicle state vector is assumed to contain the following elements
% X = [x y z r p h vx vy vz rr pr hr]'
%      1 2 3 4 5 6 7  8  9  10 11 12   index
xj  =  1; yj  =  2; zj  =  3;
rj  =  4; pj  =  5; hj  =  6;
vxj =  7; vyj =  8; vzj =  9;
rrj = 10; prj = 11; hrj = 12;

% the vehicle state X is w.r.t. local-level frame, i.e.
% X_LL North, Y_LL East, Z_LL Down
% but the nav_t structure reports positions & velocities 
% w.r.t. world frame, i.e.
% Y_W  North, X_W  East, Z_W  Down
Rwl = [0  1  0; 
       1  0  0; 
       0  0 -1];

SENTINAL = -9876543210;
for ii=1:length(slist)
  ind = find(mindx == ii);

  switch upper(slist{ii})
   case 'RDI'
    nav_t.RDI.rovtime = t(ind)';
    nav_t.RDI.altitude = SENTINAL;
    nav_t.RDI.r1 = SENTINAL;
    nav_t.RDI.r2 = SENTINAL;
    nav_t.RDI.r3 = SENTINAL;
    nav_t.RDI.r4 = SENTINAL;
    nav_t.RDI.heading = unwrap(X(hj,ind)');
    nav_t.RDI.pitch   = unwrap(X(pj,ind)');
    nav_t.RDI.roll    = unwrap(X(rj,ind)');
    vel_w = Rwl*X([vxj,vyj,vzj],ind);
    nav_t.RDI.vx = vel_w(1,:)';
    nav_t.RDI.vy = vel_w(2,:)';
    nav_t.RDI.vz = vel_w(3,:)';
    pos_w = Rwl*X([xj,yj,zj],ind);
    nav_t.RDI.nx = pos_w(1,:)';
    nav_t.RDI.ny = pos_w(2,:)';
    nav_t.RDI.nz = pos_w(3,:)';
    nav_t.RDI.cog = atan2(X(vyj,ind),X(vxj,ind))';
    nav_t.RDI.sog = sqrt(X(vxj,ind).^2 + X(vyj,ind).^2)';
    nav_t.RDI.bt_status = zeros(length(ind),1);
    nav_t.RDI.u = zeros(length(ind),1);
    nav_t.RDI.v = zeros(length(ind),1);
    nav_t.RDI.w = zeros(length(ind),1);    
    for jj=1:length(ind)
      % create body-frame velocity measurements
      Rlv = rotxyz(X([rj,pj,hj],ind(jj)));
      vel_v = Rlv'*X([vxj,vyj,vzj],ind(jj));
      nav_t.RDI.u(jj) = vel_v(1);
      nav_t.RDI.v(jj) = vel_v(2);
      nav_t.RDI.w(jj) = vel_v(3);
    end
    
   case 'PARO'
    nav_t.PARO.rovtime = t(ind)';
    nav_t.PARO.depth   = -X(zj,ind)';
    
   case 'XBOW'
    nav_t.XBOW.rovtime = t(ind)';
    nav_t.XBOW.heading = unwrap(X(hj,ind)');
    nav_t.XBOW.pitch   = unwrap(X(pj,ind)');
    nav_t.XBOW.roll    = unwrap(X(rj,ind)');
    nav_t.XBOW.hr = X(hrj,ind)';
    nav_t.XBOW.pr = X(prj,ind)';
    nav_t.XBOW.rr = X(rrj,ind)';
    
   case 'PXF'
    nav_t.PXF.rovtime = t(ind)';
    nav_t.PXF.imgnum  = [0:(length(ind)-1)]';
    for ii=1:length(ind)
      nav_t.PXF.imgname(ii,:) = sprintf('SIM.%04d.tif',nav_t.PXF.imgnum(ii));
    end
    
   otherwise
    error(sprintf('Unknown sensor %s',slist{ii}));
  end % switch
end % for ii
