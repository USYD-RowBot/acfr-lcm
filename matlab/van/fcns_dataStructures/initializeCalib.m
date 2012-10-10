function Calib = initializeCalib(calfile);
%function Calib = initializeCalib(calfile);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-31-2004      rme         Created and written.
%                                Code snippet taken from process_init.m
%                                and moved here to compartmentalize code.
%    12-18-2004      rme         Moved radial compensating information here.

% load camera calibration file
[K,k,kr,kt,imgsize] = load_camera_calib(calfile);

% compute inverse distortion model parameters based on Helsika model
a_oulu = radcomp_oulu(K,kr,kt,imgsize);

% compute a matlab tform which applies distortion model
% (i.e. adds radial and tangential distortion to pixel coordinates)
TformAddDistortion = make_rad_tform(a_oulu,K,kr,kt);

% compute a matlab tform which applies an inverse distortion model
% (i.e. removes radial and tangential distortion from pixels coordinates)
TformRemoveDistortion = fliptform(TformAddDistortion);

% compute a tmap_b data structure used within tformarray to generate
% a radial compensated image by resampling the raw image
tmap_b = create_radcomp_tmap_b(imgsize(1), imgsize(2), TformAddDistortion);

% compute matlab imtransform udata vdata
udata = [0, imgsize(2)-1];
vdata = [0, imgsize(1)-1];

% compose Calib output data structure
Calib.imgsize = imgsize;
Calib.K       = K;
Calib.k       = k;
Calib.kr      = kr;
Calib.kt      = kt;
Calib.a_oulu  = a_oulu;
Calib.udata   = udata;
Calib.vdata   = vdata;
Calib.TformAddDistortion    = TformAddDistortion;
Calib.TformRemoveDistortion = TformRemoveDistortion;
Calib.tmap_b  = tmap_b;
