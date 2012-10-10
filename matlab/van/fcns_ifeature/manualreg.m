function manualreg(imgnum1,imgnum2,nav_t,TheConfig,varargin)
%function manualreg(imgnum1,imgnum2,nav_t,TheConfig)  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-17-2003      rme         Created and written.

error(nargchk(4,6,nargin));

mindex1 = find(nav_t.PXF.imgnum == imgnum1);
mindex2 = find(nav_t.PXF.imgnum == imgnum2);

sensor = 'PHINS';
rph1 = [nav_t.(sensor).roll(nav_t.PXF.camind.RDI(mindex1)); ...
	nav_t.(sensor).pitch(nav_t.PXF.camind.RDI(mindex1)); ...
	nav_t.(sensor).heading(nav_t.PXF.camind.RDI(mindex1))];

rph2 = [nav_t.(sensor).roll(nav_t.PXF.camind.RDI(mindex2)); ...
	nav_t.(sensor).pitch(nav_t.PXF.camind.RDI(mindex2)); ...
	nav_t.(sensor).heading(nav_t.PXF.camind.RDI(mindex2))];

R1 = rotxyz(rph1);
R2 = rotxyz(rph2);

% image filename 1
tmp = dir(sprintf('%s*.%04d.%s',TheConfig.Data.imageDir,imgnum1,TheConfig.Data.imageExtension));
filename1 = tmp(1).name;
% load image 1
I1 = imread(strcat(TheConfig.Data.imageDir,filename1));

% image filename 2
tmp = dir(sprintf('%s*.%04d.%s',TheConfig.Data.imageDir,imgnum2,TheConfig.Data.imageExtension));
filename2 = tmp(1).name;
% load image 2
I2 = imread(strcat(TheConfig.Data.imageDir,filename2));

K = TheConfig.Calib.K;
if nargin == 4
  manual_corr(K,R1,R2,I1,I2);
else
  % pass thru input_points and base_points
  manual_corr(K,R1,R2,I1,I2,varargin{:});
end
