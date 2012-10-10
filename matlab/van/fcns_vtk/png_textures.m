function [texdir,numImages] = png_textures(imdir,TheConfig,downsamp)
%function [texdir,numImages] = png_textures(imdir,TheConfig,downsamp)  
%
% imdir string ends with a slash
% 20040209 OP   Created
% 20050510 rme  modified to use TheConfig

texdir = sprintf('texture%d/',downsamp);
% make dir if it doesn't exist

if ~exist([imdir,texdir],'dir')
  fprintf('texdir does not exist.\n');
  fprintf('Creating %s\n',texdir);
  mkdir(imdir,texdir);
end

% create radial mask
alpha = repmat(uint8(255),TheConfig.Calib.imgsize);
alpha = undistort_image(alpha,TheConfig.Calib.tmap_b);
alpha = imresize(alpha,1/downsamp,'bilinear');

d = dir(strcat(imdir,'/*.jpg'));
Inames = strvcat(d.name);
numImages = size(Inames,1);
for i = 1:numImages
  fprintf('processing image %i out of %i\r',i,numImages)
  I = imread(strcat(imdir,Inames(i,:)));
  %I = rgb2gray(I);
  %J = undistort_image(I,TheConfig.Calib.tmap_b);
  J = imresize(I,1/downsamp,'bilinear');
  %J = adapthisteq(J,'clipLimit',0.02,'Distribution','rayleigh');  
  imwrite(J,strcat(imdir,texdir,Inames(i,1:end-3),'png'),'png','BitDepth',8,'Alpha',alpha);
end
fprintf('\ndone!\n')

