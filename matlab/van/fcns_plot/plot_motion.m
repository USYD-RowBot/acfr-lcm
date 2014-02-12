function plot_motion(I1,u1,v1,u2,v2,PlotProps,LineProps)
%function plot_motion(I1,u1,v1,u2,v2,PlotProps,LineProps)  
%
% overlays motion vector field on images
%
% Date        Who     What
% ---------   ----    --------------
% 2003        OP      Created
% 2004-04-15  rme     Modified.

if ~exist('PlotProps','var') || isempty(PlotProps)
  PlotProps = {'x'};
end
if ~exist('LineProps','var') || isempty(LineProps)
  LineProps = {'Color','g'};
end

if ~isempty(I1)
  udata = [1 size(I1,2)] - 1;
  vdata = [1 size(I1,1)] - 1;
  imagesc(udata,vdata,I1);
  axis ij image off;
  colormap gray
else
  axis ij equal;
end

hold on;
% motion vectors from points in I1 to points in I2
line([u1';u2'],[v1';v2'],LineProps{:});
plot_multicolor(u1,v1,PlotProps{:});
hold off;
