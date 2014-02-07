function DVL = dvlrenav(t_dvl, btv, btv_status, x_vs_dvl, t_att, rph, x_vs_att, x_lr_att, ...
                         xo, declination, sigma_d, sigma_psi)
%DVLRENAV  renavigates DVL using forward Euler integration.
%   dvl = dvlrenav(t_dvl, btv, btv_status, x_vs_dvl, t_att, rph, x_vs_att, x_lr_att, ...
%                   xo, declination, sigma_d, sigma_psi)
%   returns integrated DVL vehicle position.
%
%   INPUT - use an empty array [] to specify default values
%     t_dvl        = [Mx1] vector of dvl timestamps [s]
%     btv          = [Mx4] matrix of [u,v,w,e] bottom-track velocities [m/s]
%     btv_status   = [Mx1] vector of bottom-track velocity status from RDI DVL
%     t_att        = [Nx1] vector of attitude sensor timestamps [s]
%     rph          = [Nx3] matrix of [r,p,h] ZYX Euler roll, pitch, heading [rad]
%     x_vs_dvl     = dvl sensor frame w.r.t. vehicle frame, i.e., 
%                    [6x1] vector of [x,y,z,r,p,h]'. [m & rads]
%     x_vs_att     = attitude sensor frame w.r.t. vehicle frame, i.e., 
%                    [6x1] vector of [x,y,z,r,p,h]'. [m & rads]
%     x_lr_att     = attitude sensor reference frame w.r.t. local-level frame, i.e.,
%                    [6x1] vector of [x,y,z,r,p,h]'. [m & rads]
%                    ([0,0,0,0,0,0] default)
%     xo*          = constant of integration w.r.t. local-level, i.e.,
%                    [3x1] vector of [x_o,y_o,z_o]'. [m] ([0,0,0] default)
%     declination* = magnetic declination w.r.t. true north (rad)
%     sigma_d*     = a scalar or vector of u,v instrument frame
%                    velocity std [m/s]
%     sigma_psi*   = a scalar or vector of heading std [rad]
%
%     * denotes optional input arguments
%
%
%   OUTPUT - dvl is a structure with the following fields:
%     t_abs    = absolute timestamp of position soln [s]
%     t_rel    = relative timestamp of position soln [s]
%     nx,ny,nz = integrated position w.r.t. world-frame 
%                (i.e., Xw East, Yw North, Zw Up). [m]
%     x_lv     = [L x 6] x,y,z,r,p,h integrated vehicle state w.r.t. 
%                local-level frame (i.e., Xl North, Yl East, Zl Down).
%     Sigma_o  = [L x 2 x 2] array of first-order XY covariance w.r.t. 
%                local-level frame. [m^2]
%     path2    = 2d path length [m]
%     path3    = 3d path length [m]
%     uvw_s    = instrument recorded velocity in sensor-frame. [m/s]
%     uvw_v    = body-frame velocity in vehicle frame. [m/s]
%     uvw_l    = vehicle velocity in local-level frame. [m/s]
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-28-2006      rme         Created and written.
%    07-20-2006      rme         Changed output format to Dvl struct.
%                                Added calucation of first-order XY covariance.
%    07-23-2006      rme         Only use good bottom-track data and non-zero 
%                                magnitude velocities.
%    07-24-2006      rme         Added sigma_d and simga_h to Dvl structure.
%                                Removed 3rd dim from uvw_{s,v} output.
%    08-29-2010      rme         Refactored for generic input argument syntax

if ~argchk('x_vs_dvl'); x_vs_dvl = [0,0,0,[180,0,135]*DTOR]'; end; % iver rdi default
if ~argchk('x_vs_att'); x_vs_att = [0,0,0,[0,0,180]*DTOR]'; end; % iver microstrain default
if ~argchk('x_lr_att'); x_lr_att = [0,0,0,[0,0,0]*DTOR]'; end; % iver microstrain default
if ~argchk('xo'); xo = [0,0,0]'; end;
if ~argchk('declination'); declination = 0; end;
if ~argchk('sigma_d'); sigma_d = 0.003; end;     % [m/s] for a 1200kHz @ 1m/s
if ~argchk('sigma_psi'); sigma_psi = 2*DTOR; end;

M = length(t_dvl);
N = length(t_att);
ii = 1:M;

% throw out bad bottom-track data
btv_status = uint8(btv_status);
btv_amp_low = bitget(btv_status,2) + bitget(btv_status,4) + bitget(btv_status,6) + bitget(btv_status,8);
btv_corr_low = bitget(btv_status,1) + bitget(btv_status,3) + bitget(btv_status,5) + bitget(btv_status,7);

idx{1} = find(btv_amp_low);
if 1
    fprintf('Tossing %5d of %d RDI b/c of btv_amp_low\n',length(idx{1}),M);
else
    fprintf('Warning %5d of %d RDI have btv_amp_low\n',length(idx{1}),M);
    idx{1} = [];
end

idx{2} = find(btv_corr_low);
if 1
    fprintf('Tossing %5d of %d RDI b/c of btv_corr_low\n',length(idx{2}),M);
else
    fprintf('Warning %5d of %d RDI have btv_corr_low\n',length(idx{2}),M);
    idx{2} = [];
end
idx = [idx{1}; idx{2}];
ii(idx) = [];

% throw out any all-zero velocity solutions
idx = find( (btv(ii,1) == 0) & (btv(ii,2) == 0) & (btv(ii,3) == 0) );
fprintf('Tossing %5d of %d RDI b/c of zero velocity soln\n',length(idx),M);
ii(idx) = [];

% interpolate attitude to sample timebase
L = length(ii);
METHOD = 'linear';
FILLVAL = 0;
rph(:,3) = rph(:,3) + declination;
rph_i = interp1(t_att,rph,t_dvl(ii),METHOD,FILLVAL);

% rotate sensor frame rph into vehicle-frame
x_rs_att = [zeros(3,L); ...
            rph_i'];
x_rv_att = ssc_head2tail(x_rs_att,ssc_inverse(x_vs_att));
x_lv_att = ssc_head2tail(x_lr_att,x_rv_att);

% rotate sensor-frame uvw into local-level
uvw_s = btv(ii,1:3)';
uvw_s = reshape(uvw_s,[3 1 L]); % stack in 3rd dim
R_lv  = rotxyz(x_lv_att(4,:),x_lv_att(5,:),x_lv_att(6,:));
R_vs  = rotxyz(x_vs_dvl(4),x_vs_dvl(5),x_vs_dvl(6));
R_vs  = repmat(R_vs,[1 1 L]);   % stack in 3rd dim
uvw_v = multiprod(R_vs,uvw_s);  % velocities w.r.t. vehicle frame
uvw_l = multiprod(R_lv,uvw_v);  % velocities w.r.t. local-level frame

uvw_s = reshape(uvw_s,[3 L]);
uvw_v = reshape(uvw_v,[3 L]);
uvw_l = reshape(uvw_l,[3 L]);

% forward Euler integrate to get position in local-level
dt = diff(t_dvl(ii));
dx = [zeros(3,1), uvw_l(:,1:end-1).*repmat(dt',[3 1])];
xyz_l = cumsum(dx,2) + repmat(xo(:),[1 L]); % include constant of integration

% compute 2d and 3d path length
path2 = cumsum(sqrt(sum(dx(1:2,:).^2,1)))';
path3 = cumsum(sqrt(sum(dx.^2,1)))';

% compute first-order XY covariance estimate
Sigma_o = firstOrderXYCov(x_lv_att(6,:),uvw_v(1,:),uvw_v(2,:),dt,sigma_d,sigma_psi);

% spit out dvl data structure
DVL.t_abs = t_dvl(ii)';
DVL.t_rel = DVL.t_abs - DVL.t_abs(1);
DVL.nx =  xyz_l(2,:); % Alvin xy
DVL.ny =  xyz_l(1,:);
DVL.nz = -xyz_l(3,:);
DVL.x_lv = [xyz_l; x_lv_att(4:6,:)];
DVL.Sigma_o = Sigma_o;
DVL.sigma_d = sigma_d;
DVL.sigma_h = sigma_psi*RTOD;
DVL.declination = declination*RTOD;
DVL.path2 = path2';
DVL.path3 = path3';
DVL.uvw_s = uvw_s;
DVL.uvw_v = uvw_v;
DVL.uvw_l = uvw_l;
DVL.x_vs_dvl = x_vs_dvl;
DVL.x_vs_att = x_vs_att;
DVL.x_lr_att = x_lr_att;

%===========================================================================================
function Sigma_o = firstOrderXYCov(psi,u,v,dt,sigma_d,sigma_psi);
% first-order XY covariance based upon u,v,h statistics
% (ignores pitch and roll)

n = length(dt) + 1;

% put into vector format if not all ready
u = u(:);
v = v(:);
dt = dt(:);
sigma_d = sigma_d(:);
sigma_psi = sigma_psi(:);

% expand scalars into vectors
if (numel(sigma_d) == 1);
  sigma_d = repmat(sigma_d,[n,1]);
end;
if (numel(sigma_psi) == 1);
  sigma_psi = repmat(sigma_psi,[n,1]);
end;

% only elements 1 through n-1 are used for forward Euler integration
u = u(1:n-1);
v = v(1:n-1);
psi = psi(1:n-1);
sigma_d = sigma_d(1:n-1);
sigma_psi = sigma_psi(1:n-1);

% stack into 3rd dim
u = reshape(u,[1,1,n-1]);
v = reshape(v,[1,1,n-1]);
psi = reshape(psi,[1,1,n-1]);
dt = reshape(dt,[1,1,n-1]);
sigma_d = reshape(sigma_d,[1,1,n-1]);
sigma_psi = reshape(sigma_psi,[1,1,n-1]);

% compute XY DVL odometry covariance
Rprime = [ sin(psi), cos(psi); ...
          -cos(psi), sin(psi)];

UV = [u.^2, u.*v; ...
      u.*v, v.^2];

I = repmat(eye(2),[1,1,n-1]);

Sigma_o = multiprod(I,dt.^2.*sigma_d.^2.*(1+sigma_psi.^2)) + ... % isotropic component
          multiprod(multiprod(multiprod(Rprime,UV),multitransp(Rprime)),dt.^2.*sigma_psi.^2); % anisotropic
Sigma_o = cat(3,zeros(2),Sigma_o);
Sigma_o = cumsum(Sigma_o,3);
