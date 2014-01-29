function S3 = structcat(S1,S2,dim)
%STRUCTCAT  Concatenate structures.
%   S3 = STRUCTCAT(S1,S2,DIM) concatenates structures S1 and S2 along
%   dimension DIM.  If unspecified, DIM defaults to 1.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-30-2006      rme         Created and written from a function of the
%                                same name written by Dana Yoerger.

if (nargin < 3);
   dim=1;
end;

fields = fieldnames(S1);
for ii = 1:length(fields);
  field = fields{ii};
  if (dim == 1);
    S3.(field) = [S1.(field); S2.(field)];
  else
    S3.(field) = [S1.(field), S2.(field)];
  end
end
