function varargout = figure(varargin)
%FIGURE Create figure window.
%   This is a wrapper to the builtin function FIGURE.  It provides a
%   workaround for MATLAB 7 so that minimized figure windows are not
%   forced to the foreground when FIGURE is called.  This behavior is 
%   consistent with the behavior of FIGURE in previous releases of 
%   MATLAB prior to MATLAB 7.
%  
%   FIGURE, by itself, creates a new figure window, and returns
%   its handle.
% 
%   FIGURE(H) makes H the current figure, forces it to become visible,
%   and raises it above all other figures on the screen.  If Figure H
%   does not exist, and H is an integer, a new figure is created with
%   handle H.
%
%   GCF returns the handle to the current figure.
%
%   Execute GET(H) to see a list of figure properties and
%   their current values. Execute SET(H) to see a list of figure
%   properties and their possible values.
%
%   See also SUBPLOT, AXES, GCF, CLF.
%
%  
% History
% DATE          WHO                      WHAT
%----------    ------------------        ------------------------------
% 2006-01-11   Ryan Eustice              Created & written.


% disable name conflict warning message
orig = warning('off','MATLAB:dispatcher:nameConflict');

if (nargin == 1) && ishandle(varargin{1});
  set(0,'CurrentFigure',varargin{1}); % make specified figure current
else;
  builtin('figure',varargin{:});
end;

% restore original warning state
warning(orig.state,orig.identifier);

% return handle
if nargout > 0;
  varargout{1} = gcf;
end;
