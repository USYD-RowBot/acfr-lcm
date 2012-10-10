function [xyzFix,bused] = lblFixAuto(Param);
%function [xyzFix,bused] = lblfixAuto(Param);
%
% Param is an input parameter structure with the following fields:
%   owtt_rovtime   is a n-vector of rovtime.
%   owtt           is a n x 4 array of recorded one-way-travel-times.
%   depth_rovtime  is a m-vector of rovtime.
%   depth          is a m-vector of pressure measured depth [m].
%   rph_rovtime    is a k-vector of rovtime.
%   rph            is a k x 3 vector of vehicle Euler roll,pitch,heading [rad].
%   x_vs_depth     is the 6-vector of [x,y,z,r,p,h] sensor offset for the
%                  depth sensor.
%   x_vs_ducer     is the 6-vector sensor offset for the transducer
%   btat           is a 4-vector of beacon turn around times, if not
%                  specified it defaults to 5ms for each beacon.
%   bxyz           is a 3 x 4 array of XYZ beacon positions in world frame
%                  (i.e., East, North, Up)
%   abcd           is a 4-vector defining the index of which beacons 
%                  correspond to A,B,C,D respectively; use 0 as an index to
%                  ignore selected beacon(s).
%   cwflag         is a flag indicating which side of the baseline to use
%                  where the baseline is defined as A->B,  +1 CW, -1 CCW.
%   sos            is the speed-of-sound to use (assumed constant) [m/s].
%
% OUTPUTS:
%   xyzFix  is a n x 3 array of XYZ LBL fixes.
%   bused   is a n x 4 array of 0's and 1's indicating which beacons
%           where used to compute the fix.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-05-2006      rme         Created and written.

n = size(Param.owtt,1);
m = size(Param.depth,1);
k = size(Param.rph,1);

% set default beacon turn-around-time to 5ms
if ~isfield(Param,'btat'); Param.btat = 5e-3*[1,1,1,1]; end;

% interpolate pressure depth to LBL times
depthi = interp1(Param.depth_rovtime,Param.depth,Param.owtt_rovtime);

% interpolate vehicle rph to LBL times
rphi = interp1(Param.rph_rovtime,Param.rph,Param.owtt_rovtime);

xyzFix = zeros(n,3);
bused  = zeros(n,4);
for ii=1:n;
  % compute depth at the LBL transducer
  x_vp = Param.x_vs_depth;                 % paro frame w.r.t. vehicle-frame
  x_vd = Param.x_vs_ducer;                 % ducer frame w.r.t. vehicle-frame
  x_lp = [0, 0, depthi(ii), rphi(ii,:)]';  % paro frame w.r.t. local-level
  x_lv = head2tail(x_lp, inverse(x_vp));   % vehicle frame w.r.t. local-level
  x_ld = head2tail(x_lv, x_vd);            % ducer frame w.r.t. local-level
  z    = -x_ld(3);                         % depth of LBL transducer

  % index that selects only valid (i.e., > 0) travel times and orders beacons A,B,C,D
  abcd = Param.abcd((Param.owtt(ii,:) > 0) & (Param.abcd > 0));
  XYZ  = Param.bxyz(:,abcd);                % reorder beacon positions accordingly
  owtt = Param.owtt(ii,abcd)-Param.btat(:,abcd)/2; % OWTTs corrected for beacon turn-around times
  Rslant = owtt*Param.sos;                 % slant ranges
  nBeacons = length(abcd);                 % number of valid beacons this fix
  
  switch nBeacons;
  case 2;
   [xyzFix(ii,1),xyzFix(ii,2),soln] = lblFix2B(XYZ,Rslant,z,Param.cwflag);
   xyzFix(ii,3) = z;
   bused(ii,abcd) = soln;
  case 3;
   [xyzFix(ii,1),xyzFix(ii,2)] = lblFixLS(XYZ,Rslant,z);
   xyzFix(ii,3) = z;
   bused(ii,abcd) = 1;
  otherwise;
   % indeterminant
   xyzFix(ii,:) = [NaN, NaN, z];
  end; % switch

end;
