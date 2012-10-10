I =imread(['/files1/data/bermuda02/Images/i20020827/' ...
	   '20020827_2149.navsurv/GrayProcessed_8/PXG.20020827.22005893.0200.tif']);

I = double(I);

sigma_d = 2; % derivation scale
sigma_i = 5; % integration scale
hsize_d = ceil(7*sigma_d);
if ~mod(hsize_d,2)
  hsize_d = hsize_d+1; % use an odd size filter
end
hsize_i = ceil(7*sigma_i);
if ~mod(hsize_i,2)
  hsize_i = hsize_i+1; % use an odd size filter
end

if false
  tic;
  gauss_d = fspecial('gaussian',hsize_d,sigma_d);
  gauss_i = fspecial('gaussian',hsize_i,sigma_i);
  Id1 = conv2(gauss_d,I);
  disp(toc);

  tic;
  m = size(I,1)+size(gauss_d,1)-1;
  n = size(I,2)+size(gauss_d,2)-1;
  gauss_d = fspecial('gaussian',hsize_d,sigma_d);
  gauss_d_fft = fft2(gauss_d,m,n);
  I_fft = fft2(I,m,n);
  Id2_fft = gauss_d_fft.*I_fft;
  Id2 = real(ifft2(Id2_fft));
  disp(toc);
end

if true
  gauss_d = fspecial('gaussian',hsize_d,sigma_d);
  if true
  [u1,v1,m1] = harris(conv2(I,gauss_d,'same'),2000, ...
  		      'hsize',hsize_i,'sigma',sigma_i, ...
  		      'tile',[6,6],'subpixel','disp');
  end
  if true
  [u2,v2,m2] = harris(conv2(I,gauss_d,'same'),2000, ...
		      'hsize',hsize_i,'sigma',sigma_i, ...
		      'tile',[6,6],'subpixel','disp','fft');
  end
end
