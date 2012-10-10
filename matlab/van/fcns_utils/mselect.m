function [varargout] = mselect(H);
%MSELECT manually delete outlier data from a plot window.
%   MSELECT by itself allows the user to select rectangular regions of
%   interest (ROI) in the current plot window.  To define a ROI the user
%   mouse clicks to define the two opposing corners of the rectangle.
%   Points falling within the ROI are then removed from the graph for
%   visual feedback.  To end the session, simply double-click anywhere
%   within the plot axes.  To undo the previous ROI simply left-click 
%   followed immediatley by a right-click.  This can be repeated until
%   until no more undos exist.
%
%   [II,ROI] = MSELECT(H) allows the user to pass optional input and output
%   arguments to MSELECT.  H is a handle to the desired plot window, which 
%   defaults to the current axis if unspecified.  II contain the indicies
%   of the outlier data points stored in vector form.  ROI is a K-element
%   structure that contains the parameters of each region of interest:
%   ROI.rectangle = [x,y,w,h] % defines the ROI rectangle
%   ROI.bounds = [xmin,xmax,ymin,ymax] % defines the ROI bounding box
%   ROI.ii % is the linear index of data points within the ROI
%
%   Example:
%   % generate a random data set with two outlier clusters
%   x = rand(100,1); 
%   x(x>0.75) =  5;
%   x(x<0.25) = -5;
%   plot(x,'.');
%   % manually delete the outlier points
%   mselect;
%
%   Dependencies: rubberband.m, getopt.m
%
%-----------------------------------------------------------------
%    History:
%    Date            Who              What
%    -----------     ------------     ----------------------------
%    02-15-2006      Ryan Eustice     Created and written.
%    07-23-2006      rme              Updated to make hfig come to foreground.


% parse args
if ~exist('H','var') || isempty(H);
  H = gca;
else;
  if ~ishandle(H);
    error('H is not a valid graphics handle.');
  end;
end;

hfig = get(H,'Parent');
hplot = get(H,'Children');
if length(hplot) == 0;
  error('No plot handles exist.');
elseif length(hplot) > 1;
  error('More than one plot handle present.');
end;


% store a copy of the original plot data
xo = get(hplot,'Xdata');
yo = get(hplot,'Ydata');
% working copy
xx = xo;
yy = yo;

% bring figure to foreground
figure(hfig);

roi = [];
done = false;
while ~done;
  % let the user define the region of interest (ROI)
  [x,y,w,h] = rubberband(H,'-return','rectangle');
  xmin = x; xmax = x+w;
  ymin = y; ymax = y+h;
  
  % check user action
  if (xmin==xmax) && (ymin==ymax); % double-click occured

    switch get(hfig,'SelectionType');
    case 'alt'; % mouse right single-click
     % delete last ROI
     if isempty(roi);
       uiwait(msgbox('No more undos!',[mfilename,'.m'],'modal'));
     else;
       % restore data points
       ii = roi(end).ii;
       xx(ii) = xo(ii);
       yy(ii) = yo(ii);
       set(hplot,'Xdata',xx,'Ydata',yy);
       drawnow;
       % purge roi
       roi(end) = [];
     end;
    otherwise;
     % end ROI selection
     done = true;
    end;
    continue;
  end;
  
  % ROI bounds
  bounds = [xmin,xmax,ymin,ymax];
  % find data within ROI
  ii = find((xmin <= xo) & (xo <= xmax) & (ymin <= yo) & (yo <= ymax));
  
  % delete data points
  hrect = [];
  xx(ii) = NaN;
  yy(ii) = NaN;
  set(hplot,'Xdata',xx,'Ydata',yy);
  drawnow;
    
  % store ROI params
  if isempty(roi);
    kk = 1;
  else;
    kk = length(roi)+1;
  end;
  roi(kk).rectangle = [x,y,w,h];
  roi(kk).bounds = bounds;
  roi(kk).ii = ii;
  
end; % while ~done

% concatenate ROI indicies
ii = unique([roi(:).ii]);

% return arguments
if nargout >= 1;
  varargout{1} = ii;
end;
if nargout == 2;
  varargout{2} = roi;
end;
