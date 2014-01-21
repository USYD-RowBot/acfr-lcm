function varargout = bathy_t_struct(action,varargin);
%function varargout = bathy_t_struct(action,varargin);
%
%   bathy_t_struct('push',nav_t,mindex_t,TheConfig);
%   The main idea behind "push" is to decompose the dvl beam ranges into the 
%   local-level coordinate frame and store them.  
%
%   bathy_t = bathy_t_struct('pop',TheConfig);
%   The main idea behind "pop" is to to project the range points into the
%   current camera frame and return a data structure specific to that image.
%
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-31-2004      rme         Created and written.
%                                Moved scene bathymetry code from
%                                Features_struct.m to here.
%    04-05-2004      rme         Restructured bathy_t structure to have
%                                'sel' field
%    11-14-2004      rme         Completely rewrote to use filtered vehicle state
%                                for placing pings in space.  Old method used raw RDI
%                                data which resulted in inconsistency.
%    01-04-2005      rme         Added caching of altitude percentile calculations used
%                                with link hypothesis calculation.
%    01-09-2006      rme         Fixed an error in projection of bathy xyz_l into
%                                a locally-level camera frame for altitude thresholding

global TheJournal;

% create a local ring buffer to hold xyz projected RDI range measurements for the 4 beams
persistent RingBuffer;
if isempty(RingBuffer);
  n = 4*200;
  RingBuffer.n     = n;
  RingBuffer.k     = 1;
  RingBuffer.xyz_l = zeros(4,n);
  RingBuffer.bnum  = zeros(1,n);
end;

% shorthand index
Xv_i = TheJournal.Index.Xv_i;
Xp_i = TheJournal.Index.Xp_i;


switch lower(action);
case 'push';
 % The main idea behind "push" is to decompose the dvl beam ranges into the 
 % local-level coordinate frame and store them.
 %----------------------------------------------------------------
 % parse args
 nav_t    = varargin{1};
 mindex_t = varargin{2};
 TheConfig = varargin{3};
 
 % grab the filtered vehicle pose
 if TheConfig.Estimator.inferenceEif;
   x_lv = TheJournal.Eif.mu(Xv_i(Xp_i));
 else;
   x_lv = TheJournal.Ekf.mu(Xv_i(Xp_i));
 end;
 Tlv = pvec2tmat(x_lv);

 % doppler to vehicle static xform
 Tvd = TheConfig.SensorXform.RDI.Tvs;
  
 % rdi raw beam range measurements
 r1 = nav_t.RDI.r1(mindex_t.RDI);
 r2 = nav_t.RDI.r2(mindex_t.RDI);
 r3 = nav_t.RDI.r3(mindex_t.RDI);
 r4 = nav_t.RDI.r4(mindex_t.RDI);
 
 % decompose into local-level
 [xyz_l,bnum] = decompose_dvl_beams_into_ll(Tlv*Tvd,r1,r2,r3,r4,100);

 % push the data onto the RingBuffer
 k = RingBuffer.k;
 for ii=1:length(bnum)
   RingBuffer.xyz_l(:,k) = xyz_l(:,ii);
   RingBuffer.bnum(k)    = bnum(ii);
   k = k+1;
   if k > RingBuffer.n; k = 1; end;
 end
 RingBuffer.k = k;
 
case 'pop';
 % The main idea behind "pop" is to to project the range points into the
 % current camera frame and return a data structure specific to that image.
 %--------------------------------------------------------------------
 TheConfig = varargin{1};

 % grab the filtered vehicle pose
 if TheConfig.Estimator.inferenceEif;
   x_lv = TheJournal.Eif.mu(Xv_i(Xp_i));
 else;
   x_lv = TheJournal.Ekf.mu(Xv_i(Xp_i));
 end;
 Tlv = pvec2tmat(x_lv);
 
 % camera coordinate xform
 Tvc = TheConfig.SensorXform.PXF.Tvs;
 
 % 3D points from ring buffer
 ind = find(RingBuffer.bnum > 0);
 xyz_l = RingBuffer.xyz_l(:,ind);
 bnum  = RingBuffer.bnum(ind)';
 
 % map 3D points into a camera frame looking straight down.
 % the Z-depth in this frame provides an altitude measurement.
 Tvo = Tvc;
 Tlo = Tlv*Tvo;
 Tlo(1:3,1:3) = eye(3); % locally-level camera frame
 Tol = Tlo^-1;
 xyz_o = Tol*xyz_l;
 ind  = find(xyz_o(3,:) < TheConfig.Data.maxAltitude);
 xyz_o = xyz_o(:,ind);
 bnum = bnum(ind);
 
 % map 3D points into camera frame
 Tlc = Tlv*Tvc;
 Tcl = Tlc^-1;
 xyz_c = Tcl*xyz_l(:,ind); 
 
 % project dvl points onto image plane
 K = TheConfig.Calib.K;
 [u,v] = dehomogenize(K*xyz_c(1:3,:));

 % calculate distortion corrected bathymetry image points
 uv_c = tformfwd([u,v],TheConfig.Calib.TformRemoveDistortion);
 u_c = uv_c(:,1);
 v_c = uv_c(:,2);
 
 % find beams which fall within image
 udata = TheConfig.Calib.udata;
 vdata = TheConfig.Calib.vdata;
 selu = (udata(1) <= u) & (u <= udata(2));
 selv = (vdata(1) <= v) & (v <= vdata(2));
 isel  = find(selu & selv);
 if isempty(isel);
   setTerminal('warning');
   fprintf('***%s: Warning! No RDI range points fall within image\n',mfilename);
   setTerminal('restore');
 end;
 
 % stuff scene depth in a structure
 bathy_t.X     = xyz_c(1,:)';
 bathy_t.Y     = xyz_c(2,:)';
 bathy_t.Z     = xyz_c(3,:)';
 bathy_t.alt   = xyz_o(3,:)';
 bathy_t.Cov_Z = repmat(TheConfig.SensorNoise.RDI.altitude,size(bathy_t.Z));
 bathy_t.bnum  = bnum;
 bathy_t.u     = u;
 bathy_t.v     = v;
 bathy_t.uc    = u_c;
 bathy_t.vc    = v_c;
 bathy_t.isel  = isel;
 bathy_t.altmin= min(bathy_t.alt);
 bathy_t.alt10 = prctile(bathy_t.alt,10);
 bathy_t.alt50 = prctile(bathy_t.alt,50);
 bathy_t.alt90 = prctile(bathy_t.alt,90);
 bathy_t.altmax= max(bathy_t.alt);
 
 % set output argument
 varargout{1} = bathy_t;
 
 % plot bathy points in image space
 if TheConfig.Plot.rdi_image_bathy;
   pixelplot(u,v,bnum,udata,vdata,1000);
   title('RDI bathy projected into pixel coordinates');
   set(1000,'Name','RDI bathy projected into pixel coordinates');   
   if ~isempty(isel);
     pixelplot(u(isel),v(isel),bnum(isel),udata,vdata,1001);
     title('RDI bathy projected within camera FOV');
     set(1001,'Name','RDI bathy projected within camera FOV');
   else;
     figure(1001); clf;
     title('No RDI bathy points are within camera FOV');
     set(1001,'Name','RDI bathy projected within camera FOV');     
   end;
 end;
 
otherwise; 
 error('No such method exists');
end;


%***************************************************************************
function [xyz_l,bnum] = decompose_dvl_beams_into_ll(Tld,r1,r2,r3,r4,flier);

% beam unit vectors in dvl sensor frame
persistent bhat1_d bhat2_d bhat3_d bhat4_d;
if isempty(bhat1_d);
  c = cos(30*DTOR); s = sin(30*DTOR);
  bhat1_d = [ 0;-s; c];
  bhat2_d = [ 0; s; c];
  bhat3_d = [ s; 0; c];
  bhat4_d = [-s; 0; c];
end;

% project the beam coordinates into a Euclidean sensor frame
xyz_d  = [];  % [4 x n] matrix of valid range points
bnum = [];  % beam number of valid range meas
if r1 > 0 && r1 < flier;
  xyz_d = [xyz_d, [r1*bhat1_d; 1]];
  bnum  = [bnum, 1];
end;
if r2 > 0 && r2 < flier;
  xyz_d = [xyz_d, [r2*bhat2_d; 1]];
  bnum  = [bnum, 2];
end;
if r3 > 0 && r3 < flier;
  xyz_d = [xyz_d, [r3*bhat3_d; 1]];
  bnum  = [bnum, 3];
end;
if r4 > 0 && r4 < flier;
  xyz_d = [xyz_d, [r4*bhat4_d; 1]];
  bnum  = [bnum, 4];
end;

if isempty(xyz_d);
  xyz_l = [];
  return;
else;
  % project beams returns into local-level
  xyz_l = Tld*xyz_d;
end;
