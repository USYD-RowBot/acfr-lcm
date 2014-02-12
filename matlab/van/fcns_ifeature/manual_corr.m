function manual_corr(K,R1,R2,I1,I2,varargin)
%MANUAL_CORR pick correspondences manually.
%   MANUAL_CORR(K,R1,R2,I1,I2) uses the camera calibration matrix K and the
%   orthonormal rotation matrices R1, R2 to warp each image I1, I2 to a
%   common orientation viewpoint to make establishing manual correspondences
%   easier.  I2 is the base image and I1 is the input image.  CPSELECT is
%   used as the GUI interface.  When finished picking correspondences,
%   use CPSELECT to save input_points and base_points to the workspace
%   and then exit CPSELECT.  Upon exit, a callback script
%   MCCPSELECTCALLBACK will warp the points back into the original image
%   space creating variables u1,v1,u2,v2 in the user workspace.
%
%   MANUAL_CORR(K,R1,R2,I1,I2,INPUT_POINTS,BASE_POINTS) passes previously
%   saved arrays INPUT_POINTS and BASE_POINTS to CPSELECT.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-16-2003      rme         Created and written.

error(nargchk(5,7,nargin));

% define image coordinates of upper left pixel to be (0,0)
nr = size(I1,1);
nc = size(I2,2);
udata = [1 nc]-1;
vdata = [1 nr]-1;

% create homographies which warp images to a common camera *orientation*.
% note that 3D translational offsets between the camera centers still
% causes parallax.
invK = inv(K);
Hinf1 = K*R1*invK;
Hinf2 = K*R2*invK;

% create matlab tform structures accounting for the fact that matlab uses
% post-multiply instead of pre-multiply arrays 
T_Hinf1 = maketform('projective',Hinf1');
T_Hinf2 = maketform('projective',Hinf2');

% apply warp to each image
[J1,xdata1,ydata1] = imtransform(I1,T_Hinf1,'bicubic',...
				 'udata',udata,'vdata',vdata);
[J2,xdata2,ydata2] = imtransform(I2,T_Hinf2,'bicubic',...
				 'udata',udata,'vdata',vdata);


% store transform information in a hidden figure
data.xdata1 = xdata1;
data.ydata1 = ydata1;
data.xdata2 = xdata2;
data.ydata2 = ydata2;
data.udata  = udata;
data.vdata  = vdata;
data.T_Hinf1 = T_Hinf1;
data.T_Hinf2 = T_Hinf2;
data.I1 = I1;
data.I2 = I2;
data.J1 = J1;
data.J2 = J2;
fig_hdl = figure;
set(fig_hdl,'Visible','off');
set(fig_hdl,'UserData',data);
set(fig_hdl,'Tag','manual_corr_cpselect');

% establish manual correspondences
if nargin == 5
  cp_hdl = cpselect(J1,J2);
else
  cp_hdl = cpselect(J1,J2,varargin{:});
end
set(cp_hdl,'WindowClosedCallBack','mccpselectcallback');
set(cp_hdl,'Tag','cpselect');

  
% $$$   % calculate feature points
% $$$   load('./output/20030325_1916/manual/0544-0577_orig.mat');
% $$$   u544 = input_points(:,1)-1;
% $$$   v544 = input_points(:,2)-1;
% $$$   u577p = base_points(:,1)-1;
% $$$   v577p = base_points(:,2)-1;
% $$$   % warp 577 points back to original image
% $$$   tmp = tforminv([u577p,v577p],T_Hc);
% $$$   u577 = tmp(:,1);
% $$$   v577 = tmp(:,2);
% $$$   
% $$$   figure(4);
% $$$   imshow(udata,vdata,I544,'notruesize');
% $$$   hold on; plot_multicolor(u544,v544,'+'); hold off;
% $$$   title(sprintf('I544 Manual Correspondence Set: %d pts',length(u544)));
% $$$   figure(5);
% $$$   imshow(udata,vdata,I577,'notruesize');
% $$$   hold on; plot_multicolor(u577,v577,'+'); hold off;
% $$$   title(sprintf('I577 Manual Correspondence Set: %d pts',length(u577)));
 
