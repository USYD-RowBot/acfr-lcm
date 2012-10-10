function [Features1,Features2,psel1,psel2] = ...
    insert_manual_pts(Features1,Features2,psel1,psel2,I1num,I2num,TheConfig);
%function [Features1,Features2,psel1,psel2] = ...
%    insert_manual_pts(Features1,Features2,psel1,psel2,I1num,I2num,TheConfig);
%
%  The purpose of this function is to allow the insertion of manually
%  established feature correspondences between an image pair.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-07-2003      rme         Created and written.
%    10-16-2003      rme         Modified to accept feature points
%                                variables (u1,v1) & (u2,v2)
%    10-18-2003      rme         Modified to automatically look for and
%                                establish correspondence file 
%    04-01-2004      rme         Changed TheConfig to include fields
%                                Calib & calib_t  

% I1num < I2num always since links matrices are upper triangular
filename = sprintf('%smanual/%04d-%04d.mat',TheConfig.Data.outdir,I1num,I2num);
if ~exist(filename,'file')
  % do nothing, return Features and psel unmodified
  return;
else
  % load manually established correspondences
  [Features1,Features2] = CalcFeatPair(filename,TheConfig.Calib,Features1,Features2);

  % assign putative correspondence index
  psel1 = [1:length(Features1.u)]';
  psel2 = psel1;
end

%*************************************************************************
function [Features1,Features2] = CalcFeatPair(filename,Calib,Features1,Features2)
pts_t = load(filename);  
% create feature point structure
Features1 = genFeatures(pts_t.u1,pts_t.v1,Calib,Features1);
Features2 = genFeatures(pts_t.u2,pts_t.v2,Calib,Features2);  


%*************************************************************************
function Features = genFeatures(u,v,Calib,Features)
% store radially uncompensated feature points
Features.u = u;
Features.v = v;

% compute radially compenstated feature points and store those as well
tmp = tformfwd([u,v],Calib.TformRemoveDistortion);
Features.uc = tmp(:,1);
Features.vc = tmp(:,2);

% erase feature vector information
faet_t.fv = [];
