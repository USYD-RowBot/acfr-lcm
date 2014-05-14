function reload(varargin)
%RELOAD loads a ssa_t snapshot into memory.
%   RELOAD SSA_FILENAME loads a ssa_t.mat structure into memory and
%   creates the workspace data structures:
%   TheJournal, link_t, mindex_t, nu_t
%   The user can then execute the script processLoop.m to resume
%   processing at the ssa_t structure they loaded.
%
%   Example:
%   RELOAD /files1/processed/van/output/titanic04/proc/ssa-0315.mat
%
% History
% Date          Who        Comment
% ----------    ---        -----------------------------------
% 09-02-2004    rme        Create
% 09-02-2004    rme        Added gzip compression
% 11-04-2004    rme        Modified to work with TheJournal.
% 12-20-2004    rme        Modified to work with TheBathy.
% 03-23-2005    rme        Declare global variables in 'base' WS.

% state snapshop filename & directory
ssfile = varargin{1};
outdir = fileparts(ssfile);

% uncompress to a temp file
tmpfile = [tempdir,'ssfile.mat'];
%eval(sprintf('!/bin/cp -f %s /tmp/tmp_ssfile.mat.gz',ssfile));
%eval('!/usr/bin/gunzip -f /tmp/tmp_ssfile.mat.gz');
eval(sprintf('!env gunzip --force --stdout %s > %s',ssfile,tmpfile));

% see what user specified variables we should load, o.w. load all
if nargin>=2;
  loadvars = {varargin{2:end}};
else;
  loadvars = {'TheJournal','TheBathy','stats_t','mindex_t','TheConfig'};
end;

% load ssa_t file
load(tmpfile);

% load the user specified variables into the base workspace
for ii=1:length(loadvars);
  switch loadvars{ii};
  case 'TheJournal'; 
   evalin('base','global TheJournal');
   global TheJournal;
   TheJournal = ssa_t.TheJournal;
  case 'TheBathy';
   evalin('base','global TheBathy');
   global TheBathy;
   load([outdir,'/TheBathy']);
  case 'stats_t';
   evalin('base','stats_t');
   global stats_t;
   stats_t = ssa_t.stats_t;
  case 'mindex_t';   assignin('base','mindex_t',ssa_t.mindex_t);
  case 'TheConfig';  load([outdir,'/TheConfig']); assignin('base','TheConfig',TheConfig);
  otherwise; warning(sprintf('unknown variable %s',loadvars{ii}));
  end;% switch
end;% for

%try;
%  stats_t    = ssa_t.stats_t;
%catch;
%  warning('No stats_t to load.');
%end;

%try;
%  tmp = load([outdir,'/','TheBathy.mat']);
%  TheBathy   = tmp.TheBathy;
%catch;
%  warning('No TheBathy to load.');
%end;

%try;
%  tmp = load([outdir,'/','TheConfig.mat']);
%  TheConfig   = tmp.TheConfig;
%  assignin('base','TheConfig',tmp.TheConfig);
%catch;
%  warning('No TheConfig to load.');
%end;
