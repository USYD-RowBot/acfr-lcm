function bool = argchk(varname);
%ARGCHK verifies existence of a function input argument.
%   BOOL = ARGCHK(VARNAME), inside the body of a user-defined function,
%   ARGCHK returns true if variable name, VARNAME, exists and is non-empty.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-27-2006      rme         Created and written.

bool = true;
if evalin('caller',['~exist(''',varname,''',''var'')']) || ...
   evalin('caller',['isempty(',varname,')']);
  bool = false;
end;
