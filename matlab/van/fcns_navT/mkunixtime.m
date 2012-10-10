function S = mkunixtime(S)
%MKUNIXTIME  Creates unixtime data field for old nav_t structure fields.
%   nav_t = MKUNIXTIME(nav_t) creates unixtime data field for nav_t
%   structure fields that don't have it.  The new loadseabed.m automatically
%   populates the unixtime field, this function is for old nav_t structures
%   that were created prior to the new loadseabed.m.
%
%--------------------------------------------------------------------
%   History:
%   Date            Who              What
%   -----------     ------------     ----------------------------
%   2006.07.21      Ryan Eustice     Created and written.

to = S.STARTTIME;
fields = fieldnames(S);
for k=1:length(fields);
  field = fields{k};
  if strcmp(field,'STARTTIME') || strcmp(field,'ENDTIME'); continue; end;
  S.(field).unixtime = S.(field).rovtime + to;
  S.(field) = reorderstructure(S.(field),'rovtime','unixtime');
end;
