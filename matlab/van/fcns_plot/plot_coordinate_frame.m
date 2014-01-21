function plot_coordinate_frame(R,t,label,scale,color)
%PLOT_COORDINATE_FRAME draws a set of coordinate axes.
%   PLOT_COORDINATE_FRAME(R,t) draws a coordinate frame at t
%   with orientation described by R.
%
%   PLOT_COORDINATE_FRAME(R,t,LABEL) adds subscript text to the coordinate
%   frame where LABEL is a string, i.e. 'X_{LABEL}','Y_{LABEL}','Z_{LABEL}'.
%   Specifying LABEL as the null string, i.e. '', labels the coordinate
%   axes as 'X','Y','Z'.
%
%   PLOT_COORDINATE_FRAME(R,t,LABEL,SCALE) specifies how big to draw the
%   coordinate frame in the current plot's units.  SCALE is any real
%   positive number.  Default is SCALE = 1.
%
%   PLOT_COORDINATE_FRAME(R,t,LABEL,SCALE,COLOR) where COLOR is a 3-tuple
%   string such as 'bbm' colors the X and Y axes as blue and Z axis as
%   magenta.  If COLOR is a 4-tuple such as 'bbmk', then the axes will be
%   color the same, but now the origin will be shown as a black dot.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-10-2003      rme         Created and written.
%    09-19-2003      rme         Added scale option
%    09-26-2003      rme         Added help text, and color option.

if ~exist('scale','var')
  scale = 1;
end
if ~exist('color','var')
  color = 'bbm';
end

% stacked column vectors i,j,k,origin
Aref = [scale*eye(3) zeros(3,1)];

A = [R t]*[Aref; ones(1,4)];

% plot coordinate frame w.r.t. reference frame
i = A(:,1); % x dir
j = A(:,2); % y dir
k = A(:,3); % z dir
o = A(:,4); % origin
line([o(1) i(1)]',[o(2) i(2)]',[o(3) i(3)]','Color',color(1)); % x
line([o(1) j(1)]',[o(2) j(2)]',[o(3) j(3)]','Color',color(2)); % y
line([o(1) k(1)]',[o(2) k(2)]',[o(3) k(3)]','Color',color(3)); % z
if length(color) == 4
  line(o(1),o(2),o(3),'Marker','o','MarkerSize',4, ...
       'MarkerEdgeColor',color(4),'MarkerFaceColor',color(4)); % origin
end
if exist('label','var')
  text(i(1),i(2),i(3),sprintf('X_{%s}',label));
  text(j(1),j(2),j(3),sprintf('Y_{%s}',label));
  text(k(1),k(2),k(3),sprintf('Z_{%s}',label));
end

