function TheJournal = pm_constant_acceleration_init(TheJournal,TheConfig)
%function TheJournal = pm_constant_acceleration_init(TheJournal,TheConfig)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-10-2004      rme         Created & written.
%    04-12-2004      rme         Added assembly of Q matrix
%    04-23-2004      rme         Changed Xpi to Xp_i
%    09-01-2004      rme         Modified to use TheConfig.ProcessModel field
  
% vehicle state vector
% Xv = [x u udot  y v vdot  z w wdot  r  a  adot   p  b  bdot   h  c  cdot]'
%       1 2 3     4 5 6     7 8 9     10 11 12     13 14 15     16 17 18
TheJournal.Index.Xv_i = [1:18];  % vehicle state index in augmented state vector
TheJournal.Index.Nv   = 18;      % # of elements in vehicle state vector

% 6 DOF vehicle pose
% Xp = [x_ll, y_ll, z_ll, r, p ,h]'
TheJournal.Index.Xp_i = [1 4 7 10 13 16];
TheJournal.Index.Np   = 6; % # of elements in pose vector

% extraneous vehicle states required for PM
% Xe = [u, udot, v, vdot, w, wdot, a, adot, b, bdot, c, cdot]'
TheJournal.Index.Xe_i = [2 3 5 6 8 9 11 12 14 15 17 18];
TheJournal.Index.Ne   = 12; % # of elements in extraneous vector

% vehicle state element indexes
TheJournal.Index.xyz_i     = [1 4 7];  TheJournal.Index.rph_i     = [10 13 16];
TheJournal.Index.uvw_i     = [2 5 8];  TheJournal.Index.abc_i     = [11 14 17];  
TheJournal.Index.uvw_dot_i = [3 6 9];  TheJournal.Index.abc_dot_i = [12 15 18];

% shorthand index
uvw_dot_i = TheJournal.Index.uvw_dot_i;
abc_dot_i = TheJournal.Index.abc_dot_i;

% process model noise covariance
TheJournal.Qv = spalloc(18,18,18);
TheJournal.Qv(uvw_dot_i,uvw_dot_i) = diag(TheConfig.ProcessModel.Noise.uvw_dot.^2);
TheJournal.Qv(abc_dot_i,abc_dot_i) = diag(TheConfig.ProcessModel.Noise.abc_dot.^2);
