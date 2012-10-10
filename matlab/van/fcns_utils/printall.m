function printall(fignums,figdir)
%PRINTALL multiple format printing.
%   PRINTALL(FIGNUMS,FIGDIR) prints all listed figures in the vector FIGNUMS
%   to the directory specified by FIGDIR.  Files will be named by figure number 
%   and printed as both .png and .epsc2 formats.
%
%   PRINTALL without any arguments brings all visible figures to the current directory.
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-20-2005      rme         Created and written.

if ~exist('fignums','var') || isempty(fignums);
  fignums = get(0,'children');
end;

if ~exist('figdir','var') || isempty(figdir);
  figdir = './';
end;

rootdir = pwd;
cd(figdir);
for ii=1:length(fignums)
  fig = fignums(ii);
  fname = sprintf('%d',fig);
  fprintf('%72s\r',' '); % print a blank line
  fprintf('Printing Figure %d  (%d of %d)\r',fig,ii,length(fignums));
  mprint(fig,fname,'png',300,'epsc2');
end;
fprintf('\n');
cd(rootdir);
