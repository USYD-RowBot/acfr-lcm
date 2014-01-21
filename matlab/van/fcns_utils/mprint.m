function mprint(fignum,filename,varargin)
%MPRINT multiple format printing.
%   MPRINT(FIGNUM,FILENAME,DPI1,FMT1,...,FMTM,DPIN,FMTN,...) prints the figure
%   specified by FIGNUM to file using the prefix FILENAME for each listed matlab
%   format in FMT.
%
%   Can also optionally set renderer by passing one of:
%   {'-painters','-zbuffer','-opengl','-auto' (default)}
%
%   Example: 
%   % print a jpeg file at default dpi of 150, a epsc and png at dpi of 300,
%   % and save a .fig file
%   mprint(1,'test','jpeg',300,'png','epsc','fig');
%
%   % force ouputer to be rendered with painters
%   mprint(1,'test',300,'-painters','epsc2');
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-02-2003      rme         Created and written.
%    11-04-2003      rme         Added -r Dots-per-inch flag
%    02-11-2003      rme         Precompute fig option
%    11-29-2004      rme         Added DPI argument
%    02-02-2005      rme         Added .fig option
%    02-04-2005      rme         Added renderer option.
%    02-19-2005      rme         Got rid of auto dpi filename tag.

% set default dpi
dpiopt = '-r150';

% set render to auto
renderer = '-auto';

for ii=1:length(varargin);
  arg = varargin{ii};
  if isnumeric(arg);
    dpiopt = ['-r',num2str(arg)];
  elseif ischar(arg);
    type = lower(arg);
    switch type;
    case 'fig'; % save .fig
     saveas(fignum,filename,'fig');
    case {'-painters','-zbuffer','-opengl','-auto'};
     renderer = type;
    otherwise;  % print to file
     if strcmp(renderer,'-auto');
       print(fignum,['-d',type],dpiopt,filename);
     else;
       print(fignum,renderer,['-d',type],dpiopt,filename);
     end;
    end;
  else;
    error('Arguments must either be numeric or character arrays');
  end;
end;
