function ColorOrderArray = brightColorOrder()
%function ColorOrderArray = brightColorOrder()
%
%   Returns an array suitable for the axes 'ColorOrder' property.  The
%   default Matlab color cycling scheme is a bit hard to see, this
%   changes the gains a bit in the RGB matrix to "brighten" up the
%   colors.
%
%   Ex:
%   x = 1:10;
%   y = 1:10;
%   x = [x; repmat(nan,size(x))];
%   y = [y; repmat(nan,size(y))];
%   figure;
%   subplot(121);
%   plot(x,y,'+');
%   set(gca,'Color',[0.75 0.75 0.75]);  % gray background  
%   subplot(122);
%   set(gca,'ColorOrder',brightColorOrder);
%   hold on;
%   plot(x,y,'+');
%   hold off;
%   set(gca,'Color',[0.75 0.75 0.75]);  % gray background  
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-03-2003      rme         Created and written.

% Stick with Matlab's default color cycling order, just increase the RGB
% gains a bit to produce "brighter" plot markers.
ColorOrderArray = [0 0 1; ... % b
		   0 1 0; ... % g
		   1 0 0; ... % r
		   0 1 1; ... % c
		   1 0 1; ... % m
		   1 1 0; ... % y
		   0 0 0];    % k
