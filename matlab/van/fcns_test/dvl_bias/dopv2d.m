function [vs_meas,Vr_meas,Xdot_L,Xdot_R] = dopv2d(l_T,l_B,x_o,v_o,theta,fignum)
% DOPV2D doppler measured velocity in 2D.
%
% INPUT
%   l_T is the [3x1] homogenous vehicle trajectory line
%   l_B is the [3x1] homogenous seafloor bottom line
%   x_o is the vehicle location as measured along the x-axis
%   v_o is the vehicle velocity along the line desribed by l_T
%   theta is the angle between the radial beam center and normal to l_T
%
% OUTPUT
%   vs_meas is the measured velocity in the sensor frame
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10/21/2003      rme         Created and written.
%  

  
% the point/line equation can be expressed in homogenous coordinates as:
% x'*l = 0  ==>  [x y 1] * [a b c]' = 0

% vehicle trajectory homogenous line l_T = [a_T, b_T, c_T]'
a_T = l_T(1); b_T = l_T(2); c_T = l_T(3);
% inhomogenous vector along line l_T
L_T = [b_T, -a_T]';

% seafloor bottom homogenous line l_B = [a_B, b_B, c_B]'
a_B = l_B(1); b_B = l_B(2); c_B = l_B(3);
% inhomogenous vectro along line l_B
L_B = [-b_B, a_B]';

% homogenous representation of trajectory vehicle position X_T is given by:
x_T = x_o;
y_T = -(a_T*x_T + c_T)/b_T;
X_T = [x_T, y_T, 1]';

% homogenous line of LEFT and RIGHT radial beams
l_L = cross(X_T,[-sin(theta),-cos(theta),0]');
a_L = l_L(1); b_L = l_L(2); c_L = l_L(3);
l_R = cross(X_T,[sin(theta),-cos(theta),0]');
a_R = l_R(1); b_R = l_R(2); c_R = l_R(3);
% inhomgenous vectors along radial lines
L_L = [-b_L, a_L]';
L_R = [-b_R, a_R]';

% radial beam intersection with seafloor (homogenous coordinates)
X_L = cross(l_L,l_B);
X_R = cross(l_R,l_B);

% inhomogenous represenation (Note variables that once contained
% homogenous 3-vectors now contain inhomogenous 2-vectors)
X_T = X_T(1:2)/X_T(3);
X_L = X_L(1:2)/X_L(3);
X_R = X_R(1:2)/X_R(3);

% velocity of vehicle X_T is defined to be along the line l_T therefore:
Xdot_T = v_o*[L_T/norm(L_T)];
xdot_T = Xdot_T(1); ydot_T = Xdot_T(2);

% the velocity of X_L and X_R is therefore related by
alpha = ( cos(theta) + a_T/b_T*sin(theta) );
gamma = ( a_B*sin(theta) + b_B*cos(theta) );
beta  = ( cos(theta) - a_T/b_T*sin(theta) );
mu    = (-a_B*sin(theta) + b_B*cos(theta) );

Xdot_L = -v_o*b_T*alpha/(gamma*norm(L_T)) * L_B;
Xdot_R = -v_o*b_T*beta/(mu*norm(L_T)) * L_B;

% relative radial velocity due to vehicle motion Xdot_T
vr_L = dot( Xdot_L, (X_L - X_T)/norm(X_L - X_T) );
vr_R = dot( Xdot_R, (X_R - X_T)/norm(X_R - X_T) );
Vr_meas = [vr_L, vr_R]';

% sensor frame velocity calculation is obtained from the inverse of the 
% velocity projection matrix
r_L = [-sin(theta), -cos(theta)]';
r_R = [ sin(theta), -cos(theta)]';
A = [r_L'; r_R'];
vs_meas = inv(A)*Vr_meas;

if exist('fignum','var')
  figure(fignum); clf;
  showGeometry(l_T,L_T,l_B,L_B,l_L,L_L,l_R,L_R,X_T,X_L,X_R,theta);
  grid on;
end
  
%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function showGeometry(l_T,L_T,l_B,L_B,l_L,L_L,l_R,L_R,X_T,X_L,X_R,theta)

hold on;

delta = 0.4*(X_R(1)-X_L(1));
xminus = X_L(1)-delta;
xplus  = X_R(1)+delta;

% draw trajectory line
yminus = -(l_T(1)*xminus+l_T(3))/l_T(2); 
yplus  = -(l_T(1)*xplus +l_T(3))/l_T(2); 
line([xminus;xplus],[yminus;yplus],'color','b','linewidth',3);

% draw seafloor bottom line
yminus = -(l_B(1)*xminus+l_B(3))/l_B(2); 
yplus  = -(l_B(1)*xplus +l_B(3))/l_B(2); 
line([xminus;xplus],[yminus;yplus],'color',[0.75 0.40 0.2],'linewidth',3);

% draw left radial line
line([X_T(1);X_L(1)],[X_T(2);X_L(2)],'color','r','linewidth',1)

% draw right radial line
line([X_T(1);X_R(1)],[X_T(2);X_R(2)],'color','r','linewidth',1)

% draw vehicle
plot(X_T(1),X_T(2),'ko','MarkerFaceColor','k');
text(X_T(1),X_T(2),'X_T');

% draw left and right bottom intersection points
plot(X_L(1),X_L(2),'ko','MarkerFaceColor','k');
text(X_L(1),X_L(2),'X_L');
plot(X_R(1),X_R(2),'ko','MarkerFaceColor','k');
text(X_R(1),X_R(2),'X_R');

hold off;
