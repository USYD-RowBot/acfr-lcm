
%                      D
%  -----               *---->  V
%    |               /  \
%    |              /    \       
%    |        r_L  /      \  r_R    
%    H            /    ^   \                   ground
%    |           /     |    \                    |
%    |          /      |     \                   |
%    |         /       |      \                  V
% ___|________/_______ o--------> ______________________  theta = 0

clear all;
theta = -[0:1:45]*DTOR;
H_0 = 5;
beta = 30*DTOR;
V = 1;

%  H = H_0 - tan(theta)*v*t;
dHdt =     - tan(theta)*V;

v_r_R = dHdt.*cos(theta)./cos(theta-beta);
v_r_L = dHdt.*cos(theta)./cos(theta+beta);

A = [ sin(beta), -cos(beta);
     -sin(beta), -cos(beta)];

v_v = inv(A)*[v_r_R; v_r_L];

figure(1); clf;
plot(theta*RTOD,v_r_R,'r',theta*RTOD,v_r_L,'b');
legend('v radial R','v radial L');
xlabel('theta [degrees]');
ylabel('velocity [m/s]');
grid on;
