function varargout = plot3_multicolor(x,y,z,marker,varargin)
% PLOT3_MULTICOLOR Plots each point a different color.
%   PLOT3_MULTICOLOR(X,Y,Z) plots the points in the vectors X, Y, and Z
%   while cycling through all available colors.
%
%   PLOT3_MULTICOLOR(X,Y,Z,MARKER) allows the user to optionally specify the
%   symbol used for plotting, e.g. MARKER = '+';
%
%   PLOT3_MULTICOLOR(X,Y,Z,MARKER,'PROPERTY1','VALUE1') allows the user to
%   optionally specify multiple property/value pairs.
%
%   The plot handle is an optional output argument and can be returned
%   with any of the above invocations.
%    
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-30-2003      rme         Created and written.
%    10-17-2003      rme         Replaced hold on & off with set(gca,'NextPlot','add')  

error(nargchk(3,inf,nargin));
error(nargoutchk(0,1,nargout));  
  
if ~exist('marker','var')
  marker = '.'; % default symbol
end

% reshape into row vectors
x = reshape(x, [1 length(x)]);
y = reshape(y, [1 length(y)]);
z = reshape(z, [1 length(z)]);

% append a row of NaNs
x = [x; repmat(nan,size(x))];
y = [y; repmat(nan,size(y))];
z = [z; repmat(nan,size(z))];

% set the default ColorOrder to a "brighter" color scheme
set(gca,'ColorOrder',brightColorOrder);
NextPlot = get(gca,'NextPlot');
set(gca,'NextPlot','add');
if nargin > 4
  % user must have also specified property/value pairs
  myhandle = plot3(x,y,z,marker,varargin{:});
else
  myhandle = plot3(x,y,z,marker);
end
set(gca,'NextPlot',NextPlot);
if nargout > 0
  varargout{1} = myhandle;
end
