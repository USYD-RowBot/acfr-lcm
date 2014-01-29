%test_movie0.m

% setup figure properties
fig = figure(1);
set(fig,'DoubleBuffer','on');
set(gca,'NextPlot','replace','Visible','off');

% display the original image and add it to a matlab movie object
I = double(imread('cameraman.tif'));
imagesc(I); colormap gray; axis equal;
matlab_movie(1) = getframe;

% create a blurring filter
sigma = 1;
g = fspecial('gaussian',ceil(3*sigma),sigma);

% recursively blur the image
for i=2:100
  disp(i);
  I = conv2(I,g,'same');
  imagesc(I); colormap gray; axis equal;
  
  % add the current figure to the matlab movie object
  matlab_movie(i) = getframe;
end

% play the matlab movie N times @ fps frames per second
N = 1;
fps = 12;
movie(matlab_movie,N,fps);

% convert and save matlab movie object as an AVI file
movie2avi(matlab_movie,'test_movie.avi','fps',fps);
