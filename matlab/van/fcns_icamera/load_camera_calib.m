function [K,k,kr,kt,imgsize,varargout] = load_camera_calib(fullpathname)
%LOAD_CAMERA_CALIB  Loads calibration data as computed by Matlab Toolbox.
%   [K,k,kr,kt,imgsize] = LOAD_CAMERA_CALIB(FILENAME) loads the matlab
%   calibration file and returns the formatted [3x3] linear
%   calibration matrix K, the [5x1] vectorized version k, the [3x1] vector
%   of radial distortion coefficients kr, the [2x1] vector tangential distortion
%   coefficients kt, and the variable imgsize=[nr nc].
%
%   [K,k,kr,kt,imgsize,k_std,kr_std,kt_std] = LOAD_CAMERA_CALIB(FILENAME)
%   same as above but also returns the estimated numerical error of
%   each parameter.  The numerical errors are approximately 3 times
%   the STD for each parameter.  Note that K_std(1,2) is the
%   uncertainty of the skew parameter only, where s = skew*alpha_x.
%
%   Note:
%   K = [alpha_x        s   x_o;   kr = [k1, k2, k3]'  kt = [a1, a2]'
%              0  alpha_y   y_o;
%              0        0     1]
%
%   k = [alpha_x, s, x_o, alpha_y, y_o]'
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-03-2002      rme         Created and written.
%    12-23-2002      rme         Modified to accept full path filename.
%    12-18-2004      rme         Updated to return imgsize.
%    04-25-2006      rme         Updated to use 'fileparts' instead of
%                                homebrewed 'strippath'.

%[directory,filename] = strippath(fullpathname);
%filename = strtok(filename,'.m'); % strip any trailing '.m'
[directory,filename] = fileparts(fullpathname);

% temporarily add the filename path  
addpath(directory);

% load matlab calibration script
eval(filename);

% remove path from workspace
rmpath(directory);

% compose linear camera calibration matrix
K = [fc(1) alpha_c*fc(1)  cc(1);
        0          fc(2)  cc(2);
        0              0      1];

k = [K(1,1:3), K(2,2:3)]';

% compose radial distortion coefficients
kr = [kc(1) kc(2) kc(5)]';

% compose tangential distortion coefficients
kt = [kc(3) kc(4)]';

% compose image size
imgsize = [ny nx];

if nargout > 3
  % compose linear camera calibration matrix uncertainties
  K_std = [fc_error(1) alpha_c_error cc_error(1) fc_error(2) cc_error(2)]';
  
  % compose radial distortion coefficient uncertainties
  kr_std = [kc_error(1) kc_error(2) kc_error(5)]';
  
  % compose tangential distoration coefficient uncertainties
  kt_std = [kc_error(3) kc_error(4)]';
  
  varargout{1} = K_std;
  varargout{2} = kr_std;
  varargout{3} = kt_std;
end
