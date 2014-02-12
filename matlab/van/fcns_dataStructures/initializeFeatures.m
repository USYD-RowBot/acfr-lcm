function Features = initializeFeatures(Image,x_lc,TheConfig);
%function Features = initializeFeatures(Image,x_lc,TheConfig);  
%
% Image is image data structure
% x_lc is a 6 DOF pose vector of the camera in the reference frame
% TheConfig is a configuration data structure
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-21-2003      rme         Created and written.
%    08-31-2003      rme         Added average scene depth code and
%                                argument nav_t.
%    02-17-2003      rme         Compress Features.mat files to save disk space
%    03-08-2004      rme         Calculate rough scene bathymetry using
%                                RDI beam slant range measurements
%    03-31-2004      rme         Moved RDI bathy to separate strucure
%                                called bathy_t
%    04-01-2004      rme         Changed TheConfig to include fields
%                                Calib & calib_t
%    04-13-2004      rme         Replaced cpose_t argument with x_lc
%    04-15-2004      rme         Added self-similarity calculation
%    04-21-2004      rme         Makes featdat dir if it doesn't exist
%    09-08-2004      rme         More efficient use of gunzip
%    11-05-2004      rme         Modified fprintf statements
%    11-08-2004      rme         Created stack to speed up temporal processing.
%    12-11-2004      rme         Moved core processing code to computeFeatures.m
%    12-18-2004      rme         Renamed to initializeFeatures.m
%                                Added Zernike and Sift subfields
%    05-08-2005      rme         Compartmentalized some code into loadFeatures.m

% allocate fifo stack
persistent Stack;
if isempty(Stack); 
  Stack = initializeStack(TheConfig.Data.stackSize); 
end;

% generate feature file name
featfile = sprintf('Features-%s.mat',Image.filename);
featdir  = strcat(TheConfig.Data.outdir,'featdat/');
if ~exist(featdir,'dir');
  mkdir(TheConfig.Data.outdir,'featdat');
end;
featfull = strcat(featdir,featfile);

%============================================
% CHECK STACK FOR FEAT_T STRUCTURE
%============================================
for ii=1:length(Stack);
  if isempty(Stack{ii}); continue; end;
  if Stack{ii}.imgnum == Image.imgnum;
    textcolor('dim','green','black');
    fprintf('Popping    %s.gz... ',featfile);
    % pop Features from stack
    [Stack,Features] = popStack(Stack,ii);
    fprintf('done %d Zernike %d SIFT.\n',Features.Zernike.Nf,Features.Sift.Nf);
    setTerminal('restore');
    return;
  end;
end;

%============================================
% CHECK DISK FOR FEAT_T STRUCTURE
%============================================
if exist([featfull,'.gz'],'file');
  textcolor('dim','green','black');
  Features = loadFeatures(featdir,Image.imgnum,'verbose');
  % push Features onto stack
  Stack = pushStack(Stack,Features);
  setTerminal('restore');
  return;
end;

%============================================
% COMPUTE FEAT_T AND SAVE IT TO DISK
%============================================
% compute Features
to = clock;
textcolor('dim','green','black');
fprintf('Generating %s.gz\n',featfile);
setTerminal('restore');
Features = computeFeatures(Image,x_lc,TheConfig);

% save feature structure
t1 = clock;
if ~exist(featdir,'dir')
  mkdir(pwd,featdir);
end
fprintf('Saving %s.gz...',featfile);
save(featfull,'Features');
cmd = sprintf('!env gzip %s',featfull); eval(cmd);
fprintf('done, %.3fs\n',etime(clock,t1));

% print stats
fprintf('TOTAL %0.3fs\n',etime(clock,to));

% push Features onto stack
Stack = pushStack(Stack,Features);
