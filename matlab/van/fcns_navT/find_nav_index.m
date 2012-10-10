function ind = find_nav_index(nav_t,field1,field2)
%FIND_NAV_INDEX  Finds navigation times corresponding to imagery.
%   IND = FIND_NAV_INDEX(NAV_T,FIELD1) returns an index structure IND.
%   Fields of IND correspond to nav field types.  The linear index
%   of IND corresponds to the linear index of FIELD1.
%
%   IND = FIND_NAV_INDEX(NAV_T,FIELD1,FIELD2) only searches FIELD2 of
%   NAV_T structure to find nearest measurement points to FIELD1.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-03-2003      rme         Created from find_nav_cam_index.m

if exist('field2','var')
  names = {field2};
else
  names = fieldnames(nav_t);
end

for ii=1:length(names)
  cname = char(names(ii)); %current name
  
  if isfield(nav_t.(cname),'rovtime') && ~strcmp(cname,field1)
    tes = delaunayn(nav_t.(cname).rovtime);
    ind.(cname) = dsearchn(nav_t.(cname).rovtime,tes,nav_t.(field1).rovtime);
  end
end
