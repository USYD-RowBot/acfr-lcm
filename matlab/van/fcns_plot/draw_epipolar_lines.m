function cvec = draw_epipolar_lines(F,u1,v1,fig,isize,varargin)
%DRAW_EPIPOLAR_LINES  Plots epipolar lines.
%   DRAW_EPIPOLAR_LINES(F,u1,v1,fig) F is the fundamental matrix which
%   relates the image pair, i.e.  x2 * F * x1 = 0.  u1,v1 are coordinates
%   of feature points in image 1, corresponding epipolar lines in image 2
%   are calculated and plotted ontop of figure(fig).
%   
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-06-2003      rme         Created and written from 
%                                sample_epipolar_lines.m
%    09-03-2003      rme         Added call to brightColorOrder
  


N = length(u1);

nr = isize(1);
nc = isize(2);

if length(varargin) > 0
  color = varargin{1};
end

for k = 1:N
  U1  = [u1(k),v1(k),1]';
  epl2 = F*U1; % epipolar line in image 2 corresponding to x1
  
  [xx2(:,k),yy2(:,k)] = line_bounds(0,nc-1,0,nr-1,epl2);
end

figure(fig);
if exist('color','var')
  h = line(xx2,yy2,'linewidth',1.25,'color',color);
else
  set(gca,'ColorOrder',brightColorOrder);
  h = line(xx2,yy2,'linewidth',1.25);
end

for ii=1:length(h)
  cvec(ii,:) = get(h(ii),'color');
end
