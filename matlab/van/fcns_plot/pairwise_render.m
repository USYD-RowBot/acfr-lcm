function [varargout] = pairwise_render(I1,I2,tform)
%PAIRWISE_RENDER  Displays two images on mosaic canvas.
%   Mavg = PAIRWISE_RENDER(I1,I2,T12) generates a mosaic canvas
%   containing both images I1 and I2.  T12 is a Matlab tform
%   structure warping image I2 towards I1.  Images are assumed to
%   be of the same size.  The images are averaged over their common
%   overlap.
%
%   Coordinate system is located at upper left corner of image with
%   +v down and +u to the right.  The top left pixel center is
%   assumed to be the coordinate origin (i.e. [0,0]).
%   
%   [J1,J2] = PAIRWISE_RENDER(I1,I2,T12) optionally returns the
%   warped mosaic canvas image J1, J2.
%
%   [Mavg,J1,J2] = PAIRWISE_RENDER(I1,I2,T12) same as above but
%   also returns the average intensity mosaic Mavg.
%
%   [J1,J2,xdata,ydata] = PAIRWISE_RENDER(I1,I2,T12) also
%   returns the mosaic output bounds.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-04-2002      rme         Created and written.
%    12-13-2002      rme         Center of upper left pixel is defined
%                                to be (0,0) not (0.5,0.5)
%    01-06-2003      rme         Modified to handle input images of
%                                different size and added option
%                                output arguments J1 & J2.


% warp the input image I2 towards I1 using the determined transform.
% calculate the warped image bounds so that both images are completly
% contained
[nr1,nc1] = size(I1);
[nr2,nc2] = size(I2);
inbounds1 = [1 1; nc1 nr1] - 1;
inbounds2 = [1 1; nc2 nr2] - 1;
outbounds = findbounds(tform,inbounds2);
mosaicbounds(1,:) = min(inbounds1(1,:),outbounds(1,:));
mosaicbounds(2,:) = max(inbounds1(2,:),outbounds(2,:));
xdata = mosaicbounds(:,1)';
ydata = mosaicbounds(:,2)';
J2 = imtransform(I2,tform, ...
		 'UData',[1 nc2]-1,'VData',[1 nr2]-1, ...		     
		 'XData',xdata,'YData',ydata);

% place the original I1 image into the mosaic canvas
J1 = imtransform(I1,maketform('affine',eye(3)), ...
		 'UData',[1 nc1]-1,'VData',[1 nr1]-1, ...		     
		 'XData',xdata,'YData',ydata);

if nargout == 1 || nargout == 3
  % generate a mosaic by creating an average of the two
  ind = find((J1~=0)==(J2~=0));
  Mavg = double(J1) + double(J2);
  Mavg(ind) = Mavg(ind)/2;
  
  dtype = class(I1);
  cmd = sprintf('Mavg = %s(Mavg);',dtype);
  eval(cmd);
end

% return the required output arguments
switch nargout
 case 1
  varargout{1} = Mavg;
 case 2
  varargout{1} = J1;
  varargout{2} = J2;
 case 3
  varargout{1} = Mavg;
  varargout{2} = J1;
  varargout{3} = J2;
 case 4
  varargout{1} = J1;
  varargout{2} = J2;
  varargout{3} = xdata;
  varargout{4} = ydata;
 otherwise
  error('Output arguments do not match function specification!');
end
