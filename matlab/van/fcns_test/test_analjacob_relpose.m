% use matlab symbolic tools to determine the relative pose
% measurement function and its analytic Jacobian.

syms r_vc p_vc h_vc tvc_v_x tvc_v_y tvc_v_z real;
syms r_lv1 p_lv1 h_lv1 tlv1_l_x tlv1_l_y tlv1_l_z real;
syms r_lv2 p_lv2 h_lv2 tlv2_l_x tlv2_l_y tlv2_l_z real;


% calculated relative pose of camera 1 w.r.t camera 2.
%---------------------------------------------------
% homogenous xform from camera to vehicle
tvc_v = [tvc_v_x, tvc_v_y, tvc_v_z]';
rph_vc = [r_vc, p_vc, h_vc]';
Rvc = rotxyz(rph_vc);
Hvc = [Rvc, tvc_v; 0 0 0 1];
 
% homogenous xform from vehicle 1 to local-level
tlv1_l = [tlv1_l_x, tlv1_l_y, tlv1_l_z]';
rph_lv1 = [r_lv1, p_lv1, h_lv1]';
Rlv1 = rotxyz(rph_lv1);
Hlv1 = [Rlv1, tlv1_l; 0 0 0 1];
 
% homogenous xform from vehicle 2 to local-level
tlv2_l = [tlv2_l_x, tlv2_l_y, tlv2_l_z]';
rph_lv2 = [r_lv2, p_lv2, h_lv2]';
Rlv2 = rotxyz(rph_lv2);
Hlv2 = [Rlv2, tlv2_l; 0 0 0 1];

% homogenous xform from camera 1 to local-level
Hlc1 = Hlv1*Hvc;
                                                                                
% homogenous xform from camera 2 to local-level
Hlc2 = Hlv2*Hvc;
 
% homogenous xform from camera 1 to camera 2
Hc2c1 = inv(Hlc2)*Hlc1;
 
% decompose into relative pose parameters
R = Hc2c1(1:3,1:3);
%rph = rot2rph_sym(R);
t = Hc2c1(1:3,4);
