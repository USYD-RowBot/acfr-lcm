%test_movie1.m

% setup figure properties
fig = figure(1);
set(fig,'DoubleBuffer','on');
set(gca,'NextPlot','replace','Visible','off');
I = imread('cameraman.tif');

% open avi file
mymovie = avifile('spin.avi');

% recursively blur the image
for i=0:5:360
  disp(i);
  J = imrotate(I,i,'nearest','crop');
  imagesc(J); colormap gray; axis image;
  
  % add the current figure to the avi file
  mymovie = addframe(mymovie,getframe(gca));
end

% close and save the avi file
mymovie = close(mymovie);
