function [varargout] = setwin(arg);
%SETWIN Zooms all subplots simultaneously for the current figure.
%   SETWIN with no arguments allows the user to graphically pick the 
%   lower and upper x-axis bounds to zoom in on.
%
%   [MIN,MAX] = SETWIN with no input arguments is the same as above, but
%   returns the lower and upper bounds as picked by the user.
%
%   SETWIN([MIN,MAX]) uses the specified upper and lower bounds.
%
%   SETWIN('reset') restores the subplots to their original state.

%History
%Date          Who            Comment
%----------    ------------   -----------------------------------
%              dana yoerger   Create
%2001/06/20    ryan eustice   Modify
% 2007-07-15    mvj    Allows twin to be specified in either direction.


if ~exist('arg')
   figure(gcf);
   [tt,nul] = ginput(2);
   %fprintf(2,'point to start time...\n');
   %[xmin,y] = ginput(1);
   %fprintf(2,'point to end time...\n');
   %[xmax,y] = ginput(1);
   xmin = min(tt);
   xmax = max(tt);
   %if xmax < xmin
   %  errordlg('start time must be less than end time','Error');
   %  return;
   %end
elseif isnumeric(arg) & (length(arg) == 2);
   figure(gcf);
   [xmin,xmax] = deal(arg(1), arg(2));
elseif isnumeric(arg)
   error('Unrecognized argument');
end

handle = get(gcf,'Children');
for i = 1:length(handle)
   if (strcmp(get(handle(i),'Type'),'axes') ...
         & ~strcmp(get(handle(i),'Tag'),'legend')) %skip figure legend
      if exist('xmin') & exist('xmax')
         set(handle(i),'XLim',[xmin,xmax],'XTickLabelMode','Auto','XTickMode','Auto','YLimMode','Auto');
      elseif strcmpi(arg,'reset')
         set(handle(i),'XLimMode','Auto','YLimMode','Auto');
      else
         error('Unrecognized argument');
      end
      SetTimeAxisFormat;
   end
end

if nargout == 2
  varargout(1) = {xmin};
  varargout(2) = {xmax};
elseif nargout == 1
   varargout(1) = {[xmin,xmax]};
end

return;
