function [Index,Qv] = pm_constant_velocity_init(TheConfig)
%function [Index,Qv] = pm_constant_velocity_init(TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-10-2004      rme         Created & written.
%    04-12-2004      rme         Added assembly of Q matrix
%    04-23-2004      rme         Changed Xpi to Xp_i
%    07-15-2004      rme         Reordered state elements
%    09-01-2004      rme         Modified to use TheConfig.ProcessModel field
%    11-12-2004      rme         Added Xa_i field to Index

Index = struct('Nv',[],'Np',[],'Ne',[],'Nf',[],'Naug',[],'fn',[],'featureLUT',[], ...
	       'Xf_i',[],'Xf_ii',[],'Xa_i',[],'Xv_i',[],'Xp_i',[],'Xe_i',[]);

% vehicle state vector
% Xv = [x y z  r p h  u v w  a b c]'
%       1 2 3  4 5 6  7 8 9  10 11 12
Index.Xv_i = [1:12];  % vehicle state index in augmented state vector
Index.Nv   = 12;      % # of elements in vehicle state vector

% 6 DOF vehicle pose
% Xp = [x_ll, y_ll, z_ll, r, p ,h]'
Index.Xp_i = [1:6];
Index.Np   = 6; % # of elements in pose vector

% extraneous vehicle states required for PM
% Xe = [u, v, w, a, b, c]'
Index.Xe_i = [7:12];
Index.Ne   = 6; % # of elements in extraneous vector

% vehicle state element indexes
Index.xyz_i = [1 2 3];  Index.rph_i = [4 5 6];
Index.uvw_i = [7 8 9];  Index.abc_i = [10 11 12];

% shorthand index
uvw_i = Index.uvw_i;
abc_i = Index.abc_i;

% process model noise covariance
Qv = zeros(12,12);
Qv(uvw_i,uvw_i) = diag(TheConfig.ProcessModel.Noise.uvw.^2);
Qv(abc_i,abc_i) = diag(TheConfig.ProcessModel.Noise.abc.^2);
