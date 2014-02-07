function Features = loadFeatures(srcdir,imgnum,verbose);
%function Features = loadFeatures(srcdir,imgnum,verbose);
%
% Loads a Features data structure where:
% srcdir -  is location of featdat directory
% imgnum - is the image number of the PXF file
% verbose - set to 'verbose' to print to screen
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    05-08-2005      rme         Created and written.

printIsTrue = exist('verbose','var') && strcmp(verbose,'verbose');

% find name of Features file
featstring = sprintf('%s/Features-*.%04d.*.mat.gz',srcdir,imgnum);
tmp = dir(featstring);
featfile = tmp.name;

if printIsTrue;
  fprintf('Loading    %s... ',featfile);
end;

% load it
featfull = [srcdir,'/',featfile];
feattmp = [tempdir,'Features-tmp.mat'];
cmd = sprintf('!env gunzip -c %s > %s',featfull,feattmp); eval(cmd);
tmp = load(feattmp);
Features = tmp.Features;

if printIsTrue;
  fprintf('done %d Zernike %d SIFT.\n',Features.Zernike.Nf,Features.Sift.Nf);
end;
