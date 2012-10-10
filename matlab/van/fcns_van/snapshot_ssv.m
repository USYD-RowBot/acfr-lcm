function [] = snapshot_ssv(imageNumber,TheConfig);
%function [] = snapshot_ssv(imageNumber,TheConfig);
%
% History
% Date          Who        Comment
% ----------    ---        -----------------------------------
% 2003/08/14    rme        Create
% 2003/08/15    rme        Indexing a structure of large arrays proved to
%                          be very slow in matlab.  therefore, rewrote
%                          to use a structure of smaller arrays and
%                          periodically save them.
% 2003/09/30    rme        Added imageNumber argument and camflag field
% 2004-04-07    rme        Moved ssv_t structure initalization from
%                          process_init.m to here.
% 2004-04-10    rme        Modified to archive Xv instead of Xp
% 2004-10-28    rme        Major code reorganization.
% 2004-11-03    rme        Modified to make ssv_t a global variable and write to disk.
% 2004-11-08    rme        Fixed a typo in mkdir
% 2006-01-09    rme        Fixed a bug in writeToDisk of ssv_t.mu_x
% 2006-05-02    rme        Hacked so that ssv_t saves every Nsecs if image sequence is empty.

global ssv_t TheJournal;
persistent counter to;
if isempty(counter); counter = 0; end;
if isempty(to); to = TheJournal.t; end;

% create an initial ssv_t structure
if isempty(ssv_t);
  ssv_t = initializeSSV;
  ssv_t.Index = TheJournal.Index;
end;

% shorthand index
Xv_i = TheJournal.Index.Xv_i;

% store a copy of the current vehicle state
cc                     = ssv_t.cc;
ssv_t.cc               = cc+1;
ssv_t.t(cc)            = TheJournal.t;
ssv_t.mu_x(:,cc)       = TheJournal.Ekf.mu(Xv_i);
ssv_t.Sigma_xx(:,:,cc) = TheJournal.Ekf.Sigma(Xv_i,Xv_i);
ssv_t.camflag(cc)      = imageNumber;

% write to disk
if ~isempty(TheConfig.Data.imageSequence);
  writeToDisk = imageNumber > -1;
else;
  Nsecs = 5;
  if ((ssv_t.t(cc) - ssv_t.t(1)) > Nsecs);
    writeToDisk = true;
    imageNumber = counter;
    counter = counter+1;
  else;
    writeToDisk = false;
  end;
end;
if writeToDisk;
  % first free up unused memory
  ssv_t.t(cc+1:end)            = [];
  ssv_t.mu_x(:,cc+1:end)       = [];
  ssv_t.Sigma_xx(:,:,cc+1:end) = [];
  ssv_t.camflag(cc+1:end)      = [];
  
  % save vehicle state snapshot structure associated
  % with most recently added image number
  outdir = [TheConfig.Data.outdir,'proc/'];
  if ~exist(outdir,'dir');
    mkdir(TheConfig.Data.outdir,'proc');
  end;
  outfile = sprintf('ssv-%04d',imageNumber);
  fprintf('==>%s: saving %s... ',mfilename,outfile);
  save([outdir,outfile],'ssv_t');

  % compress .mat file
  cmd = sprintf('!env gzip -f %s/%s.mat',outdir,outfile);
  eval(cmd);
  fprintf('done, crunched=%s\n',time2str(sec2hms(TheJournal.t-to),'24','hms','hms'));
  
  % reset and preallocate a new ssv_t structure
  Index = ssv_t.Index;
  ssv_t = initializeSSV;
  ssv_t.Index = Index;
end;

%====================================================================
function ssv_t = initializeSSV;

global TheJournal;

% create an initial ssv_t structure
Nmax           = 500;
Nv             = TheJournal.Index.Nv;
ssv_t.t        = zeros(1,Nmax);
ssv_t.mu_x     = zeros(Nv,Nmax);
ssv_t.Sigma_xx = zeros(Nv,Nv,Nmax);
ssv_t.camflag  = zeros(1,Nmax);
ssv_t.cc       = 1;
  
