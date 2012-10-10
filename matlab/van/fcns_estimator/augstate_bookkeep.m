function [] = augstate_bookkeep()
%function [] = augstate_bookkeep()
%
%============================================================================
% Update the state vector index data structure to reflect 
% a newly appended delayed-state
%============================================================================  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-09-2004      rme         Created from augment_state.m
%    10-14-2004      rme         Modified to *append* current robot pose to end of state vector
%    10-19-2004      rme         Added feature graph matrix fgraph and modified
%                                last element of Xf_ii to be robot indices
%    10-28-2004      rme         Major code reorganization.
%    11-02-2004      rme         Added Xa_i "all" state index
%    11-04-2004      rme         Assigned vehicle to featureLUT as "infinity"
%    12-27-2004      rme         Added path length calculation.
  
global TheJournal;

% pointers to useful variables
Nv    = TheJournal.Index.Nv;
Xvp_i = TheJournal.Index.Xv_i; % previous vehicle index x(t)
Xvc_i = Xvp_i + Nv;            % current  vehicle index x(t+1)

% update augmented state book keeping
%===========================================
% previous vehicle state x(t) has become a new map element and 
% current robot state x(t+1) is appended
TheJournal.Index.Xv_i = Xvc_i;

% increase count of map items
TheJournal.Index.Nf   = TheJournal.Index.Nf+1;

% increase count of state elements
TheJournal.Index.Naug = TheJournal.Index.Naug + Nv;

% increase count of current feature index
fn = TheJournal.Index.fn+1;
TheJournal.Index.fn   = fn;

% add camera number to feature look-up-table
TheJournal.Index.featureLUT(fn)   = TheJournal.Index.imageQueue;
TheJournal.Index.featureLUT(fn+1) = inf; % current vehicle

% store the current path length with feature number
TheJournal.Pathlen.pathvec2(fn) = TheJournal.Pathlen.pathlen2;
TheJournal.Pathlen.pathvec3(fn) = TheJournal.Pathlen.pathlen3;

% add new feature index to feature list
TheJournal.Index.Xf_ii{fn}   = Xvp_i;
TheJournal.Index.Xf_ii{fn+1} = Xvc_i;
TheJournal.Index.Xa_i = [TheJournal.Index.Xf_ii{:}];

% add new feature index to total feature list
TheJournal.Index.Xf_i(1,Xvp_i) = Xvp_i;

% update "feature" adjacency graph with a Markov connection
ii = [TheJournal.Index.fn, TheJournal.Index.fn+1];
TheJournal.Links.fgraph(ii,ii) = 1;
