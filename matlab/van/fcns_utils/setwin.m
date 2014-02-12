function varargout = setwin(varargin)
%SETWIN Zooms all subplots simultaneously for current figure.
%   SETWIN with no arguments allows the user to graphically pick the 
%   lower and upper x-axis bounds to zoom in on.
%
%   SETWIN([MIN,MAX]) uses the specified upper and lower bounds on
%   the current figure;
%
%   SETWIN(FIG) specifies the current figure to use.
%
%   SETWIN(FIG,[MIN,MAX]) uses the specified upper and lower bounds
%   on the figure specified by FIG.
%
%   SETWIN('reset') restores the current subplots to their original
%   state.
%
%   XLIM = SETWIN(...) optionally outputs the selected x-axis bounds
%   as the 2-vector XLIM.
%
%History
%Date          Who            Comment
%----------    ------------   -----------------------------------
%              dana yoerger   Create
%2001/06/20    ryan eustice   Modify
%2003/04/25    ryan eustice   Added figure option
%2006/01/05    rme            Modified output to be a 2-vector consistent
%                             with xlim.m
%2008/08/06    rme            Modified to use short-circuit && instead of &

if nargin == 0
   figure(gcf);
   fprintf('point to start time...\n');
   [xmin,y] = ginput(1);
   fprintf('point to end time...\n');
   [xmax,y] = ginput(1);
elseif nargin==1 && isnumeric(varargin{1}) && length(varargin{1})==2
   figure(gcf);
   [xmin,xmax] = deal(varargin{1}(1), varargin{1}(2));
elseif nargin==1 && isnumeric(varargin{1})
   figure(varargin{1}(1));
   fprintf('point to start time...\n');
   [xmin,y] = ginput(1);
   fprintf('point to end time...\n');
   [xmax,y] = ginput(1);
elseif nargin==2 && isnumeric(varargin{1}) && isnumeric(varargin{2})
   figure(varargin{1});
   [xmin,xmax] = deal(varargin{2}(1), varargin{2}(2));   
elseif isnumeric(varargin{1})
   error('Unrecognized argument');
end

handle = get(gcf,'Children');
for i = 1:length(handle)
   if (strcmp(get(handle(i),'Type'),'axes') ...
         && ~strcmp(get(handle(i),'Tag'),'legend')) %skip figure legend
      if exist('xmin','var') && exist('xmax','var')
         set(handle(i),'XLim',[xmin,xmax],'XTickLabelMode','Auto','XTickMode','Auto','YLimMode','Auto');   
      elseif strcmpi(varargin{1},'reset')
         set(handle(i),'XLimMode','Auto','YLimMode','Auto');
      else
         error('Unrecognized argument');
      end
   end
end

if nargout > 0
  varargout{1} = [xmin, xmax];
end
