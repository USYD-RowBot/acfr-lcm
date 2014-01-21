function render_cameras(R,t,frame,scale,color)
%RENDER_CAMERAS draws two relative camera coordinate frames.
%   RENDER_CAMERAS(R,t) draws two camera coordinate frames where R,t
%   are defined such that the camera projection matrices are:
%   P1 = K[I | 0]  and P2 = K[R | t].
%
%   RENDER_CAMERAS(R,t,FRAME) allows control of which camera to consider as
%   the reference frame, default is FRAME = 2.
%
%   RENDER_CAMERAS(R,t,FRAME,SCALE) allows control over how "big" to draw
%   the coordinate frames.  Default is SCALE = 1; see PLOT_COORDINATE_FRAME
%   for details.
%
%   RENDER_CAMERAS(R,t,FRAME,SCALE,COLOR) allows control over the
%   coloring of the coordinate axes.  Default is COLOR = 'ggmk'; see
%   PLOT_COORDINATE_FRAME for details.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-26-2003      rme         Created and written.

error(nargchk(0,5,nargin))

if ~exist('frame','var')
  frame = 2;
end
if ~exist('scale','var')
  scale = 1;
end
if ~exist('color','var')
  color = 'ggmk';
end

if frame == 2
  % w.r.t. camera 2 coordinate frame
  plot_coordinate_frame(eye(3),[0 0 0]','C2',scale,color);
  plot_coordinate_frame(R,t,'C1',scale,color);
else
  % w.r.t. camera 1 coordinate frame
  plot_coordinate_frame(eye(3),[0 0 0]','C1',scale,color);
  plot_coordinate_frame(R',-R'*t,'C2',scale,color);
end
set(gca,'Zdir','reverse','Xdir','reverse','Color',[0.8 0.8 0.8]);
