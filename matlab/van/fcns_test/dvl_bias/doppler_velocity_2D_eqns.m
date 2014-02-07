% this script uses matlab's symbolic capability to work
% out the analytical expressions for measured radial doppler velocity
% over a sloping terrain.
% 10/20/2003 rme

syms a_T b_T c_T real; % line l_T
syms a_B b_B c_B real; % line l_B
syms x_T y_T real;     % vehicle location on line l_T
syms theta real;

y_T = -(a_T*x_T + c_T)/b_T;

% vehicle trajectory is given by l_T
l_T = [a_T, b_T, c_T]';

% seafloor bottom is given by l_B
l_B = [a_B, b_B, c_B]';

% vehicle location on line l_T is given by X_T
X_T = [x_T, y_T, 1]';

% line of left radial beam is given by l_L
l_L = cross(X_T,[-sin(theta), -cos(theta), 0]');

% line of right radial beam is given by l_R
l_R = cross(X_T,[sin(theta), -cos(theta), 0]');

% left radial beam intersection with seafloor is given by X_L
X_L = cross(l_L,l_B);

% right radial beam intersection with seafloor is given by X_R
X_R = cross(l_R,l_B);
