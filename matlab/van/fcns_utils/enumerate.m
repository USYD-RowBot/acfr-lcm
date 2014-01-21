function varargout = enumerate
%ENUMERATE  enumerates a set of variables
%   [A1,A2,A3,...,AN] = ENUMERATE assigns each variable a unique value
%   similar in concept to enumeration in C.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-18-2003      rme         Created and written.
  

for ii=1:nargout
  varargout{ii} = rand;
end
