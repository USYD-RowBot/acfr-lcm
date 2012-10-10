function [H21,H21_tform] = navreg_homog(nav,camind,i1,K,R,t,I1,I2)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-13-2002      rme         Created and written.
%    12-13-2002      rme         Center of upper left pixel is defined
%                                to be (0,0) not (0.5,0.5)

%=========================================
% STATIC RIGID BODY TRANSFORMATIONS
%=========================================
% rotation matrix mapping camera frame to vehicle frame
Rvc = [0 -1  0; 
       1  0  0; 
       0  0  1];
% vector from vehicle origin to camera origin expressed 
% in vehicle frame
Tvc_v = [1.40 0 0]'; % meters

%======================================
% REFERENCE COORDINATE SYSTEM WHICH 
% MEASURES VEHICLE X,Y POSITION HAS
% Yr NORTH, Xr EAST, Zr UP
% WORLD COORDINATE I'M WORKING WITH HAS
% Xw NORTH, Yw EAST, Zw DOWN
% WORLD FRAME IS WHAT I'M MEASURING
% RPH ATTITUDE MEASUREMENTS RELATIVE TO
%--------------------------------------
% ^ Yref North      ^ Xworld  North
% |                 |
% o---> Xref  East  o---> Yworld East
%======================================
Rwr = [0  1  0; 
       1  0  0; 
       0  0 -1];

%=========================================
% VEHICLE1 POSE IN WORLD FRAME
%=========================================
% euler angles of vehicle1 relative to world frame
rph1 = [nav.RDI.roll(camind.RDI(i1));
	nav.RDI.pitch(camind.RDI(i1));
	nav.RDI.heading(camind.RDI(i1))]*pi/180;
rph1 = rph2euler(rph1);
% vector from world origin to vehicle1 origin as measured
% in world frame
Owv1_w = Rwr * [nav.RDI.nx(camind.RDI(i1)), ...
		nav.RDI.ny(camind.RDI(i1)), 0]';
Owv1_w(3) = nav.PARO.depth(camind.PARO(i1));
% rotation matrix from vehicle1 to world frame
Rwv1 = rotxyz(rph1);

%======================================
% GROUND PLANE EQUATION AS EXPRESSED
% IN WORLD COORDINATE SYSTEM
% n'*x = d
%======================================
% outward (away from camera) ground plane
% normal expressed in world coordinate frame
n_w = [0,0,1]';

% express plane normal in camera 1 frame
n_c1 = Rvc' * Rwv1' * n_w;
  
% express plane distance in camera 1 frame
d_c1 = nav.RDI.altitude(camind.RDI(i1));
  
  
% compute plane induced homography
[H21,H21_tform] = plane_induced_homog(R,t,K,K,n_c1,d_c1);
  
FLICKER = 0;
if FLICKER
  flicker(I1,I2);
  % warp I1 into I2's frame
  I1p = imtransform(I1, H21_tform, ...
		    'UData',[1 nc]-1,'VData',[1 nr]-1, ...
		    'XData',[1 nc]-1,'YData',[1 nr]-1);
  flicker(I2,I1p);
else
  %warp I2 towards I1 using the inverse of the
  %determined transform.  calculate the warped
  %image bounds so that both images are completly
  %contained
  Mavg = pairwise_render(I1,I2,fliptform(H21_tform));  
  figure(1); imshow(I1,'notruesize'); title('I1');
  figure(2); imshow(I2,'notruesize'); title('I2');  
  figure(3); imshow(Mavg,'notruesize'); title('Mavg');
end
