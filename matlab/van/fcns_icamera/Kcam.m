function K = Kcam(fm,xo,yo)
%function K = Kcam(fm,xo,yo)  
%calculates the camera calibration matrix
%inputs:
%fm : focal length in units of pixels
%xo, yo : location of principal point in image plane

K = [fm, 0,  xo; ...
     0,  fm, yo; ...
     0,  0,  1];
  
