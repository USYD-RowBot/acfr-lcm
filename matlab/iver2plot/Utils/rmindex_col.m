function S = rmindex_col(S,ii)
%RMINDEX  Removes data from structure fields.
%   S2 = RMINDEX(S1,ii) removes rows of data from the fields contained in the
%   structure S1 based upon the entries in the vector ii.  S1 must be a
%   1-by-1 structure.
%
%--------------------------------------------------------------------
%   History:
%   Date            Who              What
%   -----------     ------------     ----------------------------
%   2009.06.20      ak               copied from rmindex written by RME 

fields = fieldnames(S);
for k=1:length(fields);
  field = fields{k};
  n = size(S.(field),2);
  if (n == 1); continue; end; % single col, skip it
  try;
    colons = repmat({':'},ndims(S.(field))-1); % number of dims other than row
    S.(field)(colons{:},ii) = []; % remove selected rows
  catch;
    warning('MATLAB:rmindex','Index exceeds range for subfield "%s"',field);
  end;
end;

