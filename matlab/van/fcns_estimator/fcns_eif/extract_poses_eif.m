function [x_lvi,x_lvj,x_vc,Sigma] = extract_poses_eif(fni,fnj,TheConfig);
%function [x_lvi,x_lvj,x_vc,Sigma] = extract_poses_eif(fni,fnj,TheConfig);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-11-2004      rme         Created and written.
%    11-27-2004      rme         Modified to use CI Sigma bounds.
%    01-08-2005      rme         Moved pointers into TheJournal into appropriate
%                                if statements.
%    01-19-2005      rme         Modified to use TheJournal.Eif.SigmaBound
  
global TheJournal;

% shorthand index
Nf    = TheJournal.Index.Nf;                % number of features
Np    = TheJournal.Index.Np;                % number of pose elements
Xp_i  = TheJournal.Index.Xp_i;              % pose element index
Xa_i  = TheJournal.Index.Xa_i;              % "all" element index
Xfi_i = TheJournal.Index.Xf_ii{fni}(Xp_i);  % delayed state pose index at time t1
Xfj_i = TheJournal.Index.Xf_ii{fnj}(Xp_i);  % delayed state pose index at time t2

% extract vehicle poses
x_lvi = TheJournal.Eif.mu(Xfi_i); % 6 DOF vehicle state at time t1
x_lvj = TheJournal.Eif.mu(Xfj_i); % 6 DOF vehicle state at time t2

% extract delayed state poses from the augmented state vector corresponding
% to the vehicle pose at the time the image was taken.  
% also check if the camera to vehicle transform x_vc is a parameter in the 
% state vector and if so extract it as well.
%---------------------------------------------------------------------------
if TheConfig.Estimator.estimateCameraXform;
  % 6 DOF camera to vehicle transform is static but unknown,
  % it is being estimated within the state vector
  % NOT IMPLEMENTED WITHIN EIF YET!
  error('estimateCameraXform not implemented in EIF yet!');
else;
  % 6 DOF camera to vehicle transform is static and known
  x_vc = TheConfig.SensorXform.PXF.x_vs;
end

if TheConfig.Estimator.useSeifsDA;
  % pointers into TheJournal
  Lambda   = TheJournal.Eif.Lambda(Xa_i,Xa_i);       % information matrix
  fgraph   = TheJournal.Links.fgraph(1:Nf+1,1:Nf+1); % state topology graph

  % joint indices of the two poses
  xij_i = [Xfi_i, Xfj_i];
  
  % indices of the joint markov blanket
  mplus_i = markov_blanket(Lambda,xij_i);
  
  % indices of the shortest topological path in feature space between nodes i and j
  [pathlen,fnpath] = dijkstra(fgraph,fni,fnj);
  if pathlen > 2;
    % indices of a modified markov blanket which includes a topological path between i and j
    mplus_i = union(mplus_i,[TheJournal.Index.Xf_ii{fnpath(2:end-1)}]);
  end;
  
  % joint conditional covariance matrix associated with pose elements 
  ii = [xij_i,mplus_i];
  SigmaTmp = spdinverse(Lambda(ii,ii));
  
  % marginalize out the markov blanket elements & add zero uncertainty for x_vc transform
  Sigma = zeros(3*Np);
  Sigma(1:2*Np,1:2*Np) = SigmaTmp(1:2*Np,1:2*Np);
else;
  % use EIF Sigma bounds
  Sigma = zeros(3*Np);
  Sigma(1:2*Np,1:2*Np) = [TheJournal.Eif.SigmaBound{fni},    TheJournal.Eif.SigmaCol(Xfi_i,:); ...
		          TheJournal.Eif.SigmaCol(Xfi_i,:)', TheJournal.Eif.SigmaCol(Xfj_i,:)];
end;
