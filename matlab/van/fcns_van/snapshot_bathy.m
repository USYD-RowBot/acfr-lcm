function snapshot_bathy(TheConfig);
%function snapshot_bathy(TheConfig);
%
%History
%Date          Who        Comment
%----------    ----       -----------------------------------
%12-20-2004    rme        Created and written.

global TheBathy;

% write TheBathy data structure to disk
outdir = [TheConfig.Data.outdir,'proc/'];
if ~exist(outdir,'dir');
  mkdir(TheConfig.Data.outdir,'proc');
end;
outfile = 'TheBathy';
fprintf('==>%s: saving %s... ',mfilename,outfile);
save([outdir,outfile],'TheBathy');
fprintf('done\n');
