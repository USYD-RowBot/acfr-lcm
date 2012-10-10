function [XYZ,uv,alt,bnum,selu,selv,selin] = find_dvl_beams_in_camera_fov(mindex,ind,RDI_t,Tvc,Tvd,Twl,K,udata,vdata)
%function [XYZ,uv,alt,bnum,selu,selv,selin] = find_dvl_beams_in_camera_fov(mindex,ind,RDI_t,Tvc,Tvd,Twl,K,udata,vdata)
%
% INPUT:
% mindex is the current RDI measurement index associated with the current image
% ind is RDI indices around mindex, e.g. ind = [-50:50]
% RDI_t is the nav_t.RDI data structure
% Tvc is the camera to vehicle [4 x 4] homogenous coordinate xform
% Tvd is the doppler to vehicle [4 x 4] homogenous coordinate xform
% Twl is the local-level to world [4 x 4] homogenous coordinate xform  
% K is the [3 x 3] camera calibration matrix
% udata is a 2-vector of left-most and right-most x pixel coordinates
% vdata is a 2-vector of top-most and bottom-most y pixel coordinates
%
% OUTPUT:
% XYZ 3D coordinates of doppler beams measurements in camera frame
% uv pixel coordinates of imaged XYZ coordinates
% alt of beams in a local-level camera frame
% bnum beam number associated with XYZ measurement
% selu is true for bathy points which fall within image u coordinate bounds
% selv is true for bathy points which fall within image v coordinate bounds
% selin is true for bathy points which fall within image
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-08-2004      rme         Created and written.
%    03-11-2004      rme         Added Twl argument
%    04-05-2004      rme         Added selu, selv, and selin output args
%    11-13-2004      rme         Fixed bug if B_d was empty

% homogenous world to local-level transform
Tlw = inv(Twl);  

% doppler pose at time instant camera measurement took place
rph_ld0 = [RDI_t.roll(mindex),RDI_t.pitch(mindex),RDI_t.heading(mindex)]';
twd0_w  = [RDI_t.nx(mindex),RDI_t.ny(mindex),RDI_t.nz(mindex)]';
tld0_l  = Tlw*[twd0_w; 1]; 
tld0_l  = tld0_l(1:3);
Tld0 = [rotxyz(rph_ld0), tld0_l; 0 0 0 1]; % doppler to local-level at camera instant

% homogenous transform describing camera pose in local-level frame
Tlc0 = Tld0*inv(Tvd)*Tvc; % camera to local-level
Tc0l = inv(Tlc0);% local-level to camera

% camera projection matrix
P = K*[eye(3), zeros(3,1)];

% doppler sensor frame beam range unit vectors
c = cos(30*DTOR); s = sin(30*DTOR);
bhat1_d = [0,-s,c]'; bhat2_d = [0,s,c]'; bhat3_d = [s,0,c]'; bhat4_d = [-s,0,c]';

% search for RDI beam measurements in camera FOV
MIN_IND = 1;
MAX_IND = length(RDI_t.rovtime);
XYZ = []; uv = []; bnum = []; alt = [];
for ii=1:length(ind)
  % rdi measurement index
  n = mindex + ind(ii);
  if (n < MIN_IND) || (n > MAX_IND)
    continue; % index n is out of range, skip
  end
  
  % rdi pose at time instant doppler measurement took place
  rph_ld = [RDI_t.roll(n),RDI_t.pitch(n),RDI_t.heading(n)]'; % rdi attitude in local-level frame
  twd_w  = [RDI_t.nx(n),RDI_t.ny(n),RDI_t.nz(n)]'; % rdi pos in world frame
  tld_l  = Tlw*[twd_w; 1]; tld_l = tld_l(1:3);     % rdi pos in local-level frame
  Tld = [rotxyz(rph_ld), tld_l; 0 0 0 1]; % doppler to local-level at doppler instant

  % range measurements in doppler frame
  b1_d = RDI_t.r1(n)*bhat1_d;
  b2_d = RDI_t.r2(n)*bhat2_d;
  b3_d = RDI_t.r3(n)*bhat3_d;
  b4_d = RDI_t.r4(n)*bhat4_d;

  % homogenous representation
  B_d = [];    % [4 x x] matrix of homogenous beam coordinates
  vbeams = []; % beam number of valid range meas
  flier = 100;
  if (RDI_t.r1(n) > 0) && (RDI_t.r1(n) < flier)
    B_d = [B_d, [b1_d; 1]];
    vbeams = [vbeams; 1];
  end
  if (RDI_t.r2(n) > 0) && (RDI_t.r2(n) < flier)
    B_d = [B_d, [b2_d; 1]];
    vbeams = [vbeams; 2];    
  end
  if (RDI_t.r3(n) > 0) && (RDI_t.r3(n) < flier)
    B_d = [B_d, [b3_d; 1]];
    vbeams = [vbeams; 3];    
  end
  if (RDI_t.r4(n) > 0) && (RDI_t.r4(n) < flier)
    B_d = [B_d, [b4_d; 1]];
    vbeams = [vbeams; 4];    
  end
  
  % check if any beams were acceptable
  if isempty(B_d); continue; end
  
  % xform beams to camera frame
  B_c = Tc0l*Tld*B_d;
    
  % project beams into image frame
  U_c = P*B_c;
  [u,v] = dehomogenize(U_c);

  % rotate beams to a local-level camera frame
  % to get an "altitude" measurement
  XYZ_c = B_c(1:3,:);
  XYZ_l = Tlc0(1:3,1:3)*XYZ_c;
  
  % store camera and pixel coordinates
  XYZ  = [XYZ; XYZ_c'];
  alt  = [alt; XYZ_l(3,:)'];
  uv   = [uv; u,v];
  bnum = [bnum; vbeams];  

end % for ii=1:length(ind)

% find beams which fall within image
selu = (udata(1) <= uv(:,1)) & (uv(:,1) <= udata(2));
selv = (vdata(1) <= uv(:,2)) & (uv(:,2) <= vdata(2));
selin = selu & selv;
if any(selin) == 0
  fprintf('Warning! No RDI range points fall within image\n');
end

%pixelplot(pixel(:,1),pixel(:,2),pixel(:,3),udata,vdata,1000);
%pixelplot(uv(:,1),uv(:,2),bnum,udata,vdata,1001);

