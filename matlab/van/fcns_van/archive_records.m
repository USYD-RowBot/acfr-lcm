function archive_records(TheConfig)
%function archive_records(TheConfig)
%
%History
% Date          Who        Comment
%-----------    ---        -----------------------------------
% 11-03-2004    rme        Created & written.
% 11-10-2004    rme        Changed directory from proc to archive
% 11-11-2004    rme        Added archiving of stats_t
% 01-02-2005    rme        Added recovery of block-diag covariance matrix for EIF
% 01-10-2006    rme        Added re-assembly of ssv_t structure.

global stats_t TheJournal;

% generate version name
ii = 1;
time = clock;
available = false;
alphabet = 'abcdefghijklmnopqrstuvwxyz';
procdir = [TheConfig.Data.outdir,'proc/'];
basedir = sprintf('%sarchive.%4d%02d%02d',TheConfig.Data.outdir,time(1),time(2),time(3));
while ~available
  version = [basedir,alphabet(ii)];
  if ~exist(version,'dir')
    available = true;
  end
  ii = ii+1;
end

% recover the block-diagonal elements of the covariance matrix
% associated with the information filter
if TheConfig.Estimator.useEif;
  fprintf('==>%s: Recovering block-diagonal covariance elements ...\n',mfilename);
  TheJournal.Eif.Sigma = sigmaEif(TheJournal);
end;

% reassemble individual ssv_t snapshots into a composite ssv_t structure
fprintf('==>%s: Reassembling ssv_t ...\n',mfilename);
ssv_t = reassemble_ssv(procdir);

% archive proc directory
fprintf('==>%s: Archiving records to:\n%s ... ',mfilename,version);
cmd = sprintf('!/bin/mv -f %s %s',procdir,version);
eval(cmd);

% archive stats
cleanUpStats;
save([version,'/','results.mat'],'TheJournal','stats_t','TheConfig','ssv_t');
fprintf('done\n');
