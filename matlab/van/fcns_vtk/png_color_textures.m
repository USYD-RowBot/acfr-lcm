function nim = png_color_textures(imdir,config)
% imdir stringends with a slash
% 20040209 OP Created
texdir = strcat(imdir,'texture/')
% make dir if it doesn't exist

if exist(texdir,'dir')==0
  fprintf('texdir does not exist.\n');
  fprintf('Creating %s\n',texdir);
  mkdir(imdir,'texture');
end

[K,kr,kt] = load_camera_calib(config.data.dpath,config.data.cfile);
% generate compensation tform
T = comp_distortion_tform(kr,kt,K);

d = dir(strcat(imdir,'*.tif'));
Inames = strvcat(d.name);

numImages = size(Inames,1);

cform2lab = makecform('srgb2lab');
cform2srgb = makecform('lab2srgb');
  
for i = 1:numImages
  fprintf('processing image %i out of %i\r',i,numImages)
  I = imread(strcat(imdir,Inames(i,:)));
  I = colorconvert(I);
  
  [nr,nc,nch] = size(I);
  
  udat = [0 nc-1];
  vdat = [0 nr-1];
  xdat = udat;
  ydat = vdat;
  
  tic;
  Icomp = (imtransform(I,T,'bicubic','UData',udat,'VData',vdat,'XData',xdat,'YData',ydat));
    
  J = imresize(Icomp,[nr/2 nc/2],'bilinear');

  if size(J,3) == 3 %rgb image
    Jmask = sum(J,3); %sum along channels
  else
    Jmask = J;
  end
  
    alpha =(~(Jmask < 1));
  % remove ripple from interpolation
  alpha = bwmorph(alpha,'erode');
  alpha = double(alpha);
  
  % from matlab's help on adapthisteq
  LAB = applycform(J, cform2lab); %convert image to L*a*b color space
  L = LAB(:,:,1); % scale the values to range from 0 to 1
  LAB(:,:,1) = adapthisteq(L,'NumTiles',[8 8],'ClipLimit',0.01,'Distribution','rayleigh');
     
  K = applycform(LAB, cform2srgb); %convert back to RGB
  figure(1);imshow(K); drawnow;

  
  %J = adapthisteq(J,'clipLimit',0.02,'Distribution','rayleigh');
 
  
  imwrite(K,strcat(texdir,Inames(i,1:end-3),'png'),'png', ...
	  'BitDepth',8,'Alpha',alpha);
 
end
fprintf('\ndone!\n')

