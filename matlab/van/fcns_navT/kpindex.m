function S = kpindex(S,ii)
%KPINDEX  Keeps data in structure fields.
%   S2 = KPINDEX(S1,ii) keeps rows of data from the fields contained in the
%   structure S1 based upon the entries in the vector ii.  S1 must be a
%   1-by-1 structure.
%
%--------------------------------------------------------------------
%   History:
%   Date            Who              What
%   -----------     ------------     ----------------------------
%   2006.07.20      Ryan Eustice     Created and written.

% create an outlier index, jj
fields = fieldnames(S);
jj = 1:size(S.(fields{1}),1);
jj(ii) = [];

% remove outliers, what remains is the desired inlier set
S = rmindex(S,jj);
