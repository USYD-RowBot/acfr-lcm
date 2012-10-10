function rph = rot2rph_sym(R)
%ROT2RPH  Convert rotation matrix into Euler roll,pitch,heading.
%   RPH = ROT2RPH(R) computes 3-vector of Euler angles
%   [roll,pitch,heading] from [3x3] rotation matrix R.  Angles are
%   measured in radians.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08/04/2003      rme         Created from Smith,Self,& Cheeseman paper.
%                                Renamed LLW's implementation to rot2rph.orig.m

% heading
h = atan(R(2,1) / R(1,1));  
  
% compute cos & sin of heading
ch = cos(h);
sh = sin(h);

% pitch
p = atan((-R(3,1)) / (R(1,1)*ch + R(2,1)*sh));

% roll
r = atan((R(1,3)*sh - R(2,3)*ch) / (-R(1,2)*sh + R(2,2)*ch));

rph = [r, p, h]';
