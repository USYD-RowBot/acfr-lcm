function snapshot_TheJournal(mindex_t,TheConfig,imgnum,tag);
%function snapshot_TheJournal(mindex_t,TheConfig,imgnum,tag);  
%
%History
%Date          Who        Comment
%----------    ---        -----------------------------------
%2003/08/14    rme        Create
%2003/10/06    rme        Added link_t argument to archive
%2004-04-07    rme        Modified to write individual ssa_t.mat
%                         files for most recently augmented state rather than
%                         keep them all in memory using a single cell array.
%2004-04-10    rme        Added imgnum arugment.
%2004-05-10    rme        Added nu_t argument
%2004-09-02    rme        Added gzip compression
%2004-09-02    rme        Store link_t efficiently
%2004-10-28    rme        Major code reorganization.
%2004-11-08    rme        Fixed a type in mkdir
%2004-12-03    rme        Added cleanUpTheJournal()
%12-20-2004    rme        Moved Bathy structure out from under the TheJournal
%                         and into a new global variable TheBathy
%12-20-2004    rme        Added archival of stats_t
%01-14-2004    rme        Added tag argument.
%                         Renamed from snapshot_ssa.m to snapshot_TheJournal.m

global ssa_t TheJournal TheBathy stats_t;

% decided to not "clean-up" TheJournal and stats_t, because these data structres
% are "preallocated" by processInit.m.  the "clean-up" operation essentialy 
% removes unused preallocated elements.  however, if we "reload" one of these
% ssa_t data structures and continue processing, then matlab will have to
% grow their arrays during processing...  compressing the .mat file with
% gzip achieves similar compression ratios anyway.  the penalty we pay is
% that it takes matlab longer to write the raw .mat file to disk.
%ssa_t.TheJournal = cleanUpTheJournal(TheJournal,TheConfig);
%ssa_t.stats_t    = cleanUpStats(stats_t);
ssa_t.TheJournal = TheJournal;
ssa_t.stats_t    = stats_t;
ssa_t.mindex_t   = mindex_t;

% save augmented state snapshot structure associated
% with most recently added image number
outdir = [TheConfig.Data.outdir,'proc/'];
if ~exist(outdir,'dir')
  mkdir(TheConfig.Data.outdir,'proc');
end
switch lower(tag);
case 'before'; outfile = sprintf('ssb-%04d',imgnum);
case 'after';  outfile = sprintf('ssa-%04d',imgnum);
otherwise; error('tag must be ''before'' or ''after''!');
end;
fprintf('==>%s: saving %s... ',mfilename,outfile);
save([outdir,outfile],'ssa_t');

% compress .mat file
cmd = sprintf('!env gzip -f %s/%s.mat',outdir,outfile);
eval(cmd);
fprintf('done\n');

%==================================================================================
function TheJournal = cleanUpTheJournal(TheJournal,TheConfig);

Nf   = TheJournal.Index.Nf;
Naug = TheJournal.Index.Naug;

% ekf
if TheConfig.Estimator.useEkf;
  TheJournal.Ekf.mu    = TheJournal.Ekf.mu(1:Naug);
  TheJournal.Ekf.Sigma = TheJournal.Ekf.Sigma(1:Naug,1:Naug);
end;

% eif
if TheConfig.Estimator.useEif;
  TheJournal.Eif.mu       = TheJournal.Eif.mu(1:Naug);
  TheJournal.Eif.eta      = TheJournal.Eif.eta(1:Naug);
  TheJournal.Eif.Lambda   = usparse(TheJournal.Eif.Lambda(1:Naug,1:Naug));
  TheJournal.Eif.SigmaCol = TheJournal.Eif.SigmaCol(1:Naug,:);
end;

% links
TheJournal.Links.plinks = usparse(TheJournal.Links.plinks);
TheJournal.Links.vlinks = usparse(TheJournal.Links.vlinks);
TheJournal.Links.fgraph = usparse(TheJournal.Links.fgraph);

%==================================================================================
function stats_t = cleanUpStats(stats_t);

fields = fieldnames(stats_t);
for ii=1:length(fields);
  field = fields{ii};
  cc = stats_t.(field).cc;
  stats_t.(field).t(cc:end)          = [];
  stats_t.(field).dt(cc:end)         = [];
  stats_t.(field).imageEvent(cc:end) = [];
  stats_t.(field).Naug(cc:end)       = [];
end;
