function S2 = rmNanIndex(S1)
%rmNanIndex  Removes NaN data from structure fields.
%   S2 = rmNanIndex(S1,ii) removes rows with NaN data from all subfields
%   contained in the structure S1.  S1 must be a 1-by-1 structure.
%
%--------------------------------------------------------------------
%   History:
%   Date            Who              What
%   -----------     ------------     ----------------------------
%   2006.07.24      Ryan Eustice     Created and written.

uberindex = [];
fields = fieldnames(S1);
for k=1:length(fields);
  field = fields{k};
  n = size(S1.(field),1);
  if (n == 1); continue; end; % single row, skip it
  
  rowdata = reshape(S1.(field),n,[]); % collapse multi-dim data along rows
  ii = find(any(isnan(rowdata),2));   % linear index of rows with NaN data
  uberindex = [uberindex; ii];
end;
uberindex = unique(uberindex); % remove any duplicate row entries

% remove rows with NaN data
S2 = rmindex(S1,uberindex);

