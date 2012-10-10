function varargout = unplot(arg,Saved)
%UNPLOT Delete the most recently created graphics object(s).
%
%   UNPLOT removes the most recently created line, surface, patch, or
%   text object created in the current axes. 
%
%   UNPLOT(N) removes the N most recent. 
%
%   UNPLOT SET takes a snapshot of the objects currently in the axes. 
%   UNPLOT REVERT removes any objects drawn since the last SET.
%
%   Note: UNPLOT does not affect objects added through the figure menus
%   and buttons. 

% Copyright 2002 by Toby Driscoll (driscoll@na-net.ornl.gov).

persistent saved

% rme 12/04/2003
if exist('Saved','var')
  saved = Saved;
end

if nargin < 1
  arg = 1;
end

c = get(gca,'children');
switch lower(arg)
 case 'set'
  saved = c;
 case 'revert'
  delete( setdiff(c,saved) )
 otherwise
  if ~ischar(arg)
    delete( c(1:arg) )
  end
end

% rme 12/04/2003
if nargout
  varargout{1} = saved;
end
