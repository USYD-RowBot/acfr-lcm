function [TheConfig] = initializeTheJournal(TheConfig)
%function [TheJournal,TheConfig] = initializeTheJournal(TheConfig)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-12-2004      rme         Created & written.
%    04-23-2004      rme         Changed Xfi to Xf_i
%    09-01-2004      rme         Modified to use TheConfig.ProcessModel field  
%    09-09-2004      rme         Initalized TheJournal.Index.fn field.
%                                Added eta and Labmda fields
%    10-17-2004      rme         Added mu_hat field
%    10-18-2004      rme         Added Rchol to field
%    10-19-2004      rme         Added LambdaADF, SigmaADF (ADF Assumed Density Filtering)
%                                Made Xf_ii{end} be robot index
%    10-27-2004      rme         Renamed state_t to TheJournal & reorganized
%    10-28-2004      rme         Major code reorganization.
%    11-02-2004      rme         Added memory pre-allocation.
%    11-04-2004      rme         Assign vehicle state feature number of infinity.
%    11-18-2004      rme         Renamed TheJournal.Eif.cholinc to TheJournal.Eif.Lchol
%    11-21-2004      rme         Added fields SigmaCol & SigmBnd to TheJournal.Eif

global TheJournal;

% initialize TheJournal.Index structure based upon process model
% specified in config file
%----------------------------------------------------------
pmfhandle_str = func2str(TheConfig.ProcessModel.pmfhandle); % pm function handle string 

% the coding convention is that each process model function has an
% initalization function associated with it.  the initalization function has
% the same name as process model with '_init' appended.
initfhandle_str = strcat(pmfhandle_str,'_init');  
initfhandle = str2func(initfhandle_str);
% evaluate the init function to fill in the TheJournal.Index structure
[index_t,TheConfig.ProcessModel.Qv] = feval(initfhandle,TheConfig);

TheConfig.ProcessModel.pmfhandle_str = pmfhandle_str;
TheConfig.ProcessModel.initfhandle_str = initfhandle_str;

% allocate "feature" state vectors
index_t.Naug  = index_t.Nv; % # of elements in augmented state vector
index_t.Nf    = 0;          % # of features in aug state vector
index_t.fn    = 0;          % index of current map feature
index_t.Xf_i  = zeros(0,1); % indices of all feature states
                            % note that zeros(0,1) specifies an empty *row*
                            % vector which makes predict_pwc_eif code "cleaner"
% individual feature indexes organized by feature number where last cell element
% is defined to be robot index
index_t.Xf_ii = {index_t.Xv_i};
index_t.Xa_i  = [index_t.Xf_ii{:}]; % index of "all" elements
index_t.featureLUT = inf; % assign robot feature number of "infinity"

%============================================================
% pre-allocate memory
%============================================================
Np        = index_t.Np;                    % size of pose state
Nv        = index_t.Nv;                    % size of vehicle state
nImages   = TheConfig.Data.nImages;         % number of images to process
nLinksPerImage = 4;                        % average number of links per image
Naug      = Nv*nImages;                    % final dimension of the augmented state vector
nnzMarkov = 3*Nv^2*(nImages-1) + 2*Nv^2*2; % number of nonzero information elements due to Markov constraints
nnzCamera = Np^2*nLinksPerImage*nImages;   % number of nonzero information elements due to Camera constraints


% preallocate common EKF/EIF data structures of size Nv
TheJournal.t            = 0;
TheJournal.Ekf.mu       = zeros(Nv,1);
TheJournal.Ekf.Sigma    = zeros(Nv,Nv);
TheJournal.Eif.mu       = TheJournal.Ekf.mu;
TheJournal.Eif.eta      = TheJournal.Ekf.mu;
TheJournal.Eif.Lambda   = TheJournal.Ekf.Sigma;
TheJournal.Index        = index_t;

% path length
TheJournal.Pathlen.pathlen2 = 0;
TheJournal.Pathlen.pathlen3 = 0;
TheJournal.Pathlen.pathvec2 = zeros(nImages,1);
TheJournal.Pathlen.pathvec3 = zeros(nImages,1);
TheJournal.Pathlen.oldpos   = zeros(3,1);

% link topology
TheJournal.Links.plinks = spalloc(1,1,1);
TheJournal.Links.vlinks = TheJournal.Links.plinks;
TheJournal.Links.fgraph = TheJournal.Links.vlinks;


if TheConfig.Estimator.useDelayedStates;
  % preallocate EKF delayed state data structures of final size Naug
  if TheConfig.Estimator.useEkf;
    TheJournal.Ekf.mu      = zeros(Naug,1);
    TheJournal.Ekf.Sigma   = zeros(Naug,Naug);
  end;
  % preallocate EIF delayed state data structures of final size Naug
  if TheConfig.Estimator.useEif;
    TheJournal.Eif.mu       = TheJournal.Ekf.mu;
    TheJournal.Eif.eta      = TheJournal.Ekf.mu;
    TheJournal.Eif.Lambda   = spalloc(Naug,Naug,nnzMarkov+nnzCamera);
    TheJournal.Eif.Lchol    = [];
    TheJournal.Eif.SigmaCol = zeros(Naug,Np);
    TheJournal.Eif.SigmaBound  = {};
    TheJournal.Eif.SigmaBound0 = {};
  end;

  % common EKF/EIF link topology data structures
  TheJournal.Links.plinks = spalloc(nImages,nImages,nnztri(nImages));
  TheJournal.Links.vlinks = TheJournal.Links.plinks;
  TheJournal.Links.fgraph = spalloc(nImages,nImages,3*nImages+nLinksPerImage*nImages);
end;
