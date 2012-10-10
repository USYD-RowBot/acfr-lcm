function Image = initializeImage(imgnum,TheConfig);
%function Image = initializeImage(imgnum,TheConfig);  
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-21-2003      rme         Created and written.
%    08-27-2003      rme         Added Calib argument and radial
%                                distortion correction of imagery.
%    08-28-2003      rme         Used undistort_image which directly
%                                calls tformarray, this speeds up processing 
%    11-02-2003      rme         Added fields IrawC & IrawG to Image
%    03-05-2004      rme         Convert to uint8 before saving warped imagery as jpeg
%    04-01-2004      rme         Changed TheConfig to include fields
%                                Calib & calib_t
%    04-21-2004      rme         Makes distcomp dir if it doesn't exist
%    11-08-2004      rme         Created image stack to speed up temporal processing
%    12-18-2004      rme         Updated to move tmap_b calc to initializeRadcomp.m
%    02-07-2006      rme         Turned off Matlab7 warning msg generated
%                                during adapthisteq.m

% create stack
persistent Stack;  
if isempty(Stack);
  Stack = initializeStack(TheConfig.Data.stackSize);
end;

% check stack for a preloaded Image structure
for ii=1:length(Stack)
  if isempty(Stack{ii}); continue; end;
  if Stack{ii}.imgnum == imgnum;
    % pop Image from stack
    [Stack,Image] = popStack(Stack,ii);
    return;
  end;
end;

% image number
Image.imgnum = imgnum;
  
% image filename
tmp = dir(sprintf('%s*.%04d.%s',TheConfig.Data.imageDir,imgnum,TheConfig.Data.imageExtension));
Image.filename = tmp(1).name;
  
% load image file
Iraw = imread(strcat(TheConfig.Data.imageDir,Image.filename));
if size(Iraw,3) > 1;
  Image.IrawC = Iraw;            % store RGB color image
  Image.IrawG = rgb2gray(Iraw);  % convert to gray scale image
else;
  Image.IrawG = Iraw;            % store gray scale image
end;

% warp image to compensate for lens distortion
%---------------------------------------------------------------------
% make a directory to store warp imagery
Image.warpdir = strcat(TheConfig.Data.outdir,'radcomp/');
if ~exist(Image.warpdir,'dir');
  mkdir(TheConfig.Data.outdir,'radcomp');
end;

% save warped imagery as jpegs to save disk space, the warped imagery
% is only used for display purposes with epipolar geometry
Image.warpfile = strcat('RAD.',Image.filename(1:end-3),'jpg');

if exist(strcat(Image.warpdir,Image.warpfile),'file');
  % load previously generated warped image
  Image.Iwarp = imread(strcat(Image.warpdir,Image.warpfile));
else;
  % SLOW WAY OF IMPLEMENTING DISTORTION CORRECTION WARPING
  %Image.Iwarp = imtransform(Image.Iraw,Calib.TformRemoveDistortion,'bilinear', ...
  %		  	     'udata',Calib.udata,'vdata',Calib.vdata, ...
  %			     'xdata',Calib.udata,'ydata',Calib.vdata);
  % 2X FASTER TO IMPLEMENT DISTORTION CORRECTION WARPING USING
  % TFORMARRAY DIRECTLY!!!
  
  % warp image to compensate for lens distortion
  if isfield(Image,'IrawC');
    Image.Iwarp = undistort_image(Image.IrawC,TheConfig.Calib.tmap_b);
  else;
    orig1 = warning('query','MATLAB:intConvertNonIntVal');
    orig2 = warning('query','MATLAB:intConvertOverflow');
    warning('off','MATLAB:intConvertNonIntVal');
    warning('off','MATLAB:intConvertOverflow');
    Iclahs = adapthisteq(Image.IrawG, TheConfig.ImageFeature.clahsArgs{:});
    warning(orig1.state,'MATLAB:intConvertNonIntVal');
    warning(orig2.state,'MATLAB:intConvertOverflow');
    Image.Iwarp = undistort_image(Iclahs,TheConfig.Calib.tmap_b);
  end;
  % matlab jpg write function doesn't support 16-bit imagery so convert to 8-bit
  Image.Iwarp = im2uint8(Image.Iwarp);
  % save warped image for visualization
  imwrite(Image.Iwarp,strcat(Image.warpdir,Image.warpfile));  
end;

% push Image onto stack
Stack = pushStack(Stack,Image);
