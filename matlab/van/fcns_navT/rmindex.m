function S = rmindex(S,ii)
%RMINDEX  Removes data from structure fields.
%   S2 = RMINDEX(S1,ii) removes rows of data from the fields contained in the
%   structure S1 based upon the entries in the vector ii.  S1 must be a
%   1-by-1 structure.
%
%--------------------------------------------------------------------
%   History:
%   Date            Who              What
%   -----------     ------------     ----------------------------
%   2003.07.24      Ryan Eustice     Created and written.
%   2006.07.20      rme              Streamlined using modern Matlab syntax.
%   2006.07.24      rme              Added warning message.

fields = fieldnames(S);
for k=1:length(fields);
  field = fields{k};
  n = size(S.(field),1);
  if (n == 1); continue; end; % single row, skip it
  try;
    colons = repmat({':'},ndims(S.(field))-1); % number of dims other than row
    S.(field)(ii,colons{:}) = []; % remove selected rows
  catch;
    warning('MATLAB:rmindex','Index exceeds range for subfield "%s"',field);
  end;
end;

