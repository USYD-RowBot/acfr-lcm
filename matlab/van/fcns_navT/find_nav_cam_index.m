function ind = find_nav_cam_index(nav_t,IMG);
%FIND_NAV_CAM_INDEX  Finds navigation times corresponding to imagery.
%   IND = FIND_NAV_CAM_INDEX(NAV_T,IMG) returns an index structure IND.
%   Fields of IND correspond to nav field types.  The linear index
%   of IND corresponds to the linear image number+1.  IMG is optional and
%   defaults to PXF, DSPL is other option.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-03-2002      rme         Created and written.
%    02-27-2004      rme         Updated to include DSPL option
%    01-09-2006      rme         Commented out nav_t.(cname).rovtime length criteria

% default pixelfly, other option is DSPL
if ~exist('IMG','var') || isempty(IMG), IMG = 'PXF'; end;
  
names = fieldnames(nav_t);

for ii=1:length(names);
  cname = names{ii}; %current name

  if isfield(nav_t.(cname),'rovtime');
    % && (length(nav_t.(cname).rovtime) >= length(nav_t.(IMG).rovtime))
    tes = delaunayn(nav_t.(cname).rovtime);
    ind.(cname) = dsearchn(nav_t.(cname).rovtime,tes,nav_t.(IMG).rovtime);
  end;
end;
