function Features = computeFeatures(Image,x_lc,TheConfig);
%function Features = computeFeatures(Image,x_lc,TheConfig);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-11-2004      rme         Created, moved core processing to here.
%    12-18-2004      rme         Added Zernike & Sift feature code.
%    01-10-2006      rme         Added Image.IrawG conversion to double
%                                for Matlab7 compatability.
%                                Turned off Matlab7 warning msg generated
%                                during adapthisteq.m

% empty Zernike data structure
Zernike.Nf   = [];
Zernike.u    = [];
Zernike.v    = [];
Zernike.uc   = [];
Zernike.vc   = [];
Zernike.A_nm = [];

%============================================
% EXTRACT INTEREST POINTS
%============================================
to = clock;
fprintf('Extracting Harris interest points... ');
sigma_d = 2; % image derivation scale
sigma_i = 5; % feature integration scale
hsize_d = ceil(7*sigma_d);
hsize_d = hsize_d + rem(hsize_d+1,2); % ensure an odd size filter
hsize_i = ceil(7*sigma_i);
hsize_i = hsize_i + rem(hsize_i+1,2); % ensure an odd size filter
gauss_d = fspecial('gaussian',hsize_d,sigma_d);
% extract harris corner points from the image using an image scale and feature scale
[isubpixel,jsubpixel] =  harris(convolve2(double(Image.IrawG),gauss_d,'same'), ...
				TheConfig.ImageFeature.Nmax, ...
				'hsize', hsize_i, ...
				'sigma', sigma_i, ...
				'tile', TheConfig.ImageFeature.tile, ...
				'subpixel');

% note that the harris function returns the subpixel i,j location of
% feature points within the image.  the camera calibration coordinate
% system has the top left pixel at (0,0) *not* (1,1) like matlab's array
% indexing.  therefore subtract 1 from the feature points to get the
% appropriate feature coordinates.
[Zernike.u,Zernike.v] = deal(jsubpixel-1, isubpixel-1);
Zernike.Nf = length(Zernike.u);

% print stats
fprintf('done, %d features in %0.3fs\n',Zernike.Nf,etime(clock,to));

%==============================================
% ENCODE INTEREST POINTS WITH ZERNIKE MOMENTS
%==============================================
to = clock;
fprintf('Generating Zernike feature descriptors... ');
R_lc = rotxyz(x_lc(4:6));
Zbasis = TheConfig.Zbasis; %pointer
% extract polar image patches
[patchmat,sel] = zernikepolar_featpatch(Image.IrawG, Zernike.u, Zernike.v, Zbasis, ...
					TheConfig.Calib.TformRemoveDistortion, ...
					TheConfig.Calib.K, R_lc, TheConfig.ImageFeature.w);

% keep those features for whom a patch could be extracted
[Zernike.u,Zernike.v] = deal(Zernike.u(sel), Zernike.v(sel));
Zernike.Nf = length(Zernike.u);

% calculate distortion corrected feature points
tmp = tformfwd([Zernike.u,Zernike.v],TheConfig.Calib.TformRemoveDistortion);
[Zernike.uc,Zernike.vc] = deal(tmp(:,1), tmp(:,2));

% encode patches with normalized Zernike moments
Zernike.A_nm = zernikeMomentsPolar(patchmat,Zbasis,'xcorr');

% print stats
fprintf('done, %d features in %0.3fs\n',Zernike.Nf,etime(clock,to));

%=================================================
% EXTRACT SIFT FEATURES
%=================================================
% empty Sift data structure
Sift.Nf   = [];
Sift.u    = [];
Sift.v    = [];
Sift.uc   = [];
Sift.vc   = [];
Sift.keys = [];

% the sift executable works much better if we first compensate for the
% lighting using adaptive histogram specification
to = clock;
fprintf('adaphisteq and undistort... ');
orig1 = warning('query','MATLAB:intConvertNonIntVal');
orig2 = warning('query','MATLAB:intConvertOverflow');
warning('off','MATLAB:intConvertNonIntVal');
warning('off','MATLAB:intConvertOverflow');
Iclahs = adapthisteq(Image.IrawG, TheConfig.ImageFeature.clahsArgs{:});
warning(orig1.state,'MATLAB:intConvertNonIntVal');
warning(orig2.state,'MATLAB:intConvertOverflow');

% compensate for radial distortion
Iclahs = undistort_image(Iclahs,TheConfig.Calib.tmap_b);

% print stats
fprintf('done, %0.3fs\n',etime(clock,to));

% extract sift features
to = clock;
Sift.keys = sift(Iclahs, TheConfig.ImageFeature.Nmax);

% compensated feature point locations
% note: this is because sift was run on warped image
[Sift.uc,Sift.vc] = deal( Sift.keys.vector(2,:)', Sift.keys.vector(1,:)');
Sift.Nf = length(Sift.uc);

% distorted feature point locations
tmp = tformfwd([Sift.uc,Sift.vc],TheConfig.Calib.TformRemoveDistortion);
[Sift.u,Sift.v] = deal( tmp(:,1), tmp(:,2) );

% print stats
fprintf('done, %d features in %0.3fs\n',Sift.Nf,etime(clock,to));

% create Features structure
Features.imgnum  = Image.imgnum;
Features.Zernike = Zernike;
Features.Sift    = Sift;
