function analytical_relorient

% extract delayed state vehicle pose from the augmented state vector
% note this corresponds to the vehicle pose at the time the image was taken
%--------------------------------------------------------------------------
Xi = rand(6,1); Xi(1:3) = 10*Xi(1:3);
Xj = rand(6,1); Xj(1:3) = 10*Xj(1:3);

% calculated relative pose of camera i w.r.t camera j.
%---------------------------------------------------
tvc_v = [1.4; 0; 0];
Rvc = [0 -1 0; 1 0 0; 0 0 1];

% numerical pose of i w.r.t. j
n_ji = numerical_relpose(Xi,Xj,Rvc,tvc_v);

% analytical pose of i w.r.t. j
a_ji = analytical_relpose(Xi,Xj,Rvc,tvc_v);

%n_ji,a_ji

% numerical zpredict and jacobian
zn_ji = numerical_zpredict([Xi;Xj],Rvc,tvc_v);
Jn    = numerical_jacobian(@numerical_zpredict,[Xi;Xj],zn_ji,[],Rvc,tvc_v);

% analytical zpredict and jacobian
[za_ji,Ja] = analytical_zpredict(Xi,Xj,Rvc,tvc_v);


disp([Jn;Ja]);
format long;
disp([svd(Jn),svd(Ja)]);
format short;
keyboard;

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function z_21 = numerical_zpredict(pvec,Rvc,tvc_v)

X1 = pvec(1:6);
X2 = pvec(7:12);
p_21 = numerical_relpose(X1,X2,Rvc,tvc_v);

% normalize baseline direction
z_21 = p_21;
z_21(1:3) = unitize(z_21(1:3));





%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function p_21 = numerical_relpose(X1,X2,Rvc,tvc_v)
% homogenous xform from camera to vehicle
Hvc = [Rvc, tvc_v; 0 0 0 1];

% homogenous xform from vehicle 1 to local-level
Hlv1 = [rotxyz(X1(4:6)), X1(1:3); 0 0 0 1];

% homogenous xform from vehicle 2 to local-level
Hlv2 = [rotxyz(X2(4:6)), X2(1:3); 0 0 0 1];

% homogenous xform from camera 1 to local-level
Hlc1 = Hlv1*Hvc;

% homogenous xform from camera 2 to local-level
Hlc2 = Hlv2*Hvc;

% homogenous xform from camera 1 to camera 2
Hc2c1 = inv(Hlc2)*Hlc1;

% decompose into relative pose parameters
R = Hc2c1(1:3,1:3);
rph = rot2rph(R);
t = Hc2c1(1:3,4);
%---------------------------------------------------

% predicted relative pose of camera 1 w.r.t camera 2
%---------------------------------------------------  
p_21 = [t; rph];

% since the camera only measures the baseline direction,
% normalize the predicted translation measurement
%p_21(1:3) = unitize(p_21(1:3));
%---------------------------------------------------  

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function p_ji = analytical_relpose(Xi,Xj,Rvc,tvc_v)

tlvi_l = Xi(1:3);
Rlvi = rotxyz(Xi(4:6));

tlvj_l = Xj(1:3);
Rlvj = rotxyz(Xj(4:6));

t = Rvc'*[(Rlvj'*Rlvi-eye(3))*tvc_v + Rlvj'*(tlvi_l-tlvj_l)];

R = Rvc'*Rlvj'*Rlvi*Rvc;

%p_ji = [t; rot2rph(R)];

h = atan2(R(2,1), R(1,1));

ch = cos(h); sh = sin(h);

p = atan2(-R(3,1), R(1,1)*ch + R(2,1)*sh);

r = atan2(R(1,3)*sh - R(2,3)*ch, -R(1,2)*sh + R(2,2)*ch);

p_ji = [t; r; p; h];


%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function [z_ji,J] = analytical_zpredict(Xi,Xj,Rvc,tvc_v)

tlvi_l = Xi(1:3);
ri = Xi(4); pi = Xi(5); hi = Xi(6);
Rlvi = rotxyz([ri,pi,hi]);

tlvj_l = Xj(1:3);
rj = Xj(4); pj = Xj(5); hj = Xj(6);
Rlvj = rotxyz([rj,pj,hj]);

t = Rvc'*[(Rlvj'*Rlvi-eye(3))*tvc_v + Rlvj'*(tlvi_l-tlvj_l)];

[b,Jt] = unitize(t);


R = Rvc'*Rlvj'*Rlvi*Rvc;

h = atan2(R(2,1), R(1,1));

ch = cos(h); sh = sin(h);

p = atan2(-R(3,1), R(1,1)*ch + R(2,1)*sh);

r = atan2(R(1,3)*sh - R(2,3)*ch, -R(1,2)*sh + R(2,2)*ch);

% predicted measurement
z_ji = [b; r; p; h];

% Calculate analytical Jacobian for baseline direction
%------------------------------------------------------
J = zeros(6,12);

%db / d(tlvi_l)
J(1:3,1:3) = Jt*[Rvc'*Rlvj'];

%db / d(ri)
J(1:3,4) = Jt*[Rvc'*Rlvj'*rotz(hi)'*roty(pi)'*drotx(ri)'*tvc_v];

%db / d(pi)
J(1:3,5) = Jt*[Rvc'*Rlvj'*rotz(hi)'*droty(pi)'*rotx(ri)'*tvc_v];

%db / d(hi)
J(1:3,6) = Jt*[Rvc'*Rlvj'*drotz(hi)'*roty(pi)'*rotx(ri)'*tvc_v];

%db / d(tlvj_l)
J(1:3,7:9) = Jt*[-Rvc'*Rlvj'];

%db / d(rj)
J(1:3,10) = Jt*[Rvc'*[drotx(rj)*roty(pj)*rotz(hj)*Rlvi*tvc_v ...
		      + drotx(rj)*roty(pj)*rotz(hj)*(tlvi_l-tlvj_l)]];

%db / d(pj)
J(1:3,11) = Jt*[Rvc'*[rotx(rj)*droty(pj)*rotz(hj)*Rlvi*tvc_v ...
		      + rotx(rj)*droty(pj)*rotz(hj)*(tlvi_l-tlvj_l)]];

%db / d(hj)
J(1:3,12) = Jt*[Rvc'*[rotx(rj)*roty(pj)*drotz(hj)*Rlvi*tvc_v ...
		      + rotx(rj)*roty(pj)*drotz(hj)*(tlvi_l-tlvj_l)]];


% Calculate analytical Jacobian for relative orientation
%-------------------------------------------------------
% dR / d(ri)
dRdri = Rvc'*Rlvj'*rotz(hi)'*roty(pi)'*drotx(ri)'*Rvc;
% dR / d(pi)
dRdpi = Rvc'*Rlvj'*rotz(hi)'*droty(pi)'*rotx(ri)'*Rvc;
% dR / d(hi)
dRdhi = Rvc'*Rlvj'*drotz(hi)'*roty(pi)'*rotx(ri)'*Rvc;
% dR / d(rj)
dRdrj = Rvc'*drotx(rj)*roty(pj)*rotz(hj)*Rlvi*Rvc;
% dR / d(pj)
dRdpj = Rvc'*rotx(rj)*droty(pj)*rotz(hj)*Rlvi*Rvc;
% dR / d(hj)
dRdhj = Rvc'*rotx(rj)*roty(pj)*drotz(hj)*Rlvi*Rvc;

% dh / d(ri)
J(6,4) = deriv_h(R,dRdri);
% dh / d(pi)
J(6,5) = deriv_h(R,dRdpi);
% dh / d(hi)
J(6,6) = deriv_h(R,dRdhi);

% dp / d(ri)
J(5,4) = deriv_p(R,dRdri,h,J(6,4));
% dp / d(pi)
J(5,5) = deriv_p(R,dRdpi,h,J(6,5));
% dp / d(hi)
J(5,6) = deriv_p(R,dRdhi,h,J(6,6));

% dr / d(ri)
J(4,4) = deriv_r(R,dRdri,h,J(6,4));
% dr / d(pi)
J(4,5) = deriv_r(R,dRdpi,h,J(6,5));
% dr / d(hi)
J(4,6) = deriv_r(R,dRdhi,h,J(6,6));

% dh / d(rj)
J(6,10) = deriv_h(R,dRdrj);
% dh / d(pj)
J(6,11) = deriv_h(R,dRdpj);
% dh / d(hj)
J(6,12) = deriv_h(R,dRdhj);

% dp / d(rj)
J(5,10) = deriv_p(R,dRdrj,h,J(6,10));
% dp / d(pj)
J(5,11) = deriv_p(R,dRdpj,h,J(6,11));
% dp / d(hj)
J(5,12) = deriv_p(R,dRdhj,h,J(6,12));

% dr / d(rj)
J(4,10) = deriv_r(R,dRdrj,h,J(6,10));
% dr / d(pj)
J(4,11) = deriv_r(R,dRdpj,h,J(6,11));
% dr / d(hj)
J(4,12) = deriv_r(R,dRdhj,h,J(6,12));

%==============================================================
function dh = deriv_h(R,dR)

dh = 1/(R(1,1)^2 + R(2,1)^2) * [R(1,1)*dR(2,1) - R(2,1)*dR(1,1)];

%==============================================================
function dp = deriv_p(R,dR,h,dh)

alpha = (R(1,1)*cos(h) + R(2,1)*sin(h));
beta  = R(3,1);

dp = 1/(alpha^2+beta^2) * ...
     [-alpha*dR(3,1) + R(3,1)*[cos(h)*dR(1,1) + sin(h)*dR(2,1) + ...
       (R(2,1)*cos(h) - R(1,1)*sin(h))*dh]];

%===============================================================
function dr = deriv_r(R,dR,h,dh)

alpha = (-R(1,2)*sin(h) + R(2,2)*cos(h));
beta  = ( R(1,3)*sin(h) - R(2,3)*cos(h));

dr = 1/(alpha^2+beta^2) * ...
     [ alpha*[sin(h)*dR(1,3) - cos(h)*dR(2,3) + (cos(h)*R(1,3) + sin(h)*R(2,3))*dh] + ...
      -beta*[-sin(h)*dR(1,2) + cos(h)*dR(2,2) - (cos(h)*R(1,2) + sin(h)*R(2,2))*dh]];
