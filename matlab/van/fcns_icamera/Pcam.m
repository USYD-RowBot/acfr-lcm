function P = Pcam(K,R,C)
%function P = Pcam(K,R,C)  
%generates the camera projection matrix for an ideal pinhole camera
% P = KR[I | -C]
% K camera intrinsic parameters
% R camera rotation matrix (orientation in world)
% C camera center in world coordinates
P = K*[R -R*C];
