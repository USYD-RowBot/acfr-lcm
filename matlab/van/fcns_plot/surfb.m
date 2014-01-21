function varargout = surfb(varargin);
%SURFB Bathymetry surf.
%   SURFB is the same as SURF, except that it sets the axis scaling
%   to be equal and lights the surface for better viewing.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    02-03-2006      rme         Created and written.

h = surf(varargin{:});
shading interp;
axis equal;
grid on;
material dull;
lighting gouraud;
light;

if nargout == 1;
  varargout{1} = h;
end;
