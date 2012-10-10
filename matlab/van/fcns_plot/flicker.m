function flicker(I1,I2,Hz,duration,varargin)
%FLICKER flickers between two images.
%  FLICKER(I1,I2) flickers between the two images I1 and I2 at a
%  default rate of 2 Hz for 7 seconds.
%
%  FLICKER(I1,I2,HZ,DURATION) flickers between the two images for the
%  rate and duration specified.
%
%  FLICKER(I1,I2,HZ,DURATION,FHANDLE,OPTARGS,OPTCMDS) allows user to specify display
%  function, pass optional args, and execute optional commands.  FHANDLE is a Matlab
%  function handle, e.g. @imagesc, OPTARGS and OPTCMDS are each cell arrays of strings.
%
%  Example:
%  I1 = imread('cameraman.tif');
%  I2 = im2bw(I1);
%  % default usage, flicker internally is using @imshow
%  flicker(I1,I2);
%  % use default rate and duration, but specify imagesc and gray color map  
%  flicker(I1,I2,[],[],@imagesc,{},{'colormap gray','axis image'});
%
% History
% DATE         WHO            WHAT
%===========   ===========    ====================
% 2002-10-22   rme            Created and written.
% 2004-12-10   rme            Added optional display arguments.

% defaults
if ~exist('Hz','var') || isempty(Hz); Hz = 2; end;
if ~exist('duration','var') || isempty(duration); duration = 7; end;
% parse args
fhandle = @imshow; if nargin >= 5; fhandle = varargin{1}; end;
optargs = {};      if nargin >= 6; optargs = varargin{2}; end;
optcmds = {};      if nargin == 7; optcmds = varargin{3}; end;

% initalization
state = 0;
Image{1} = I1;
Image{2} = I2;

% bring current figure to foreground
figure(gcf);
dborig = get(gcf,'DoubleBuffer');
set(gcf,'DoubleBuffer','on');

% flicker between images I1 and I2
to = clock;
while etime(clock,to) < duration;
  feval(fhandle,Image{state+1},optargs{:});  % display image
  for ii=1:length(optcmds);                  % execute any optional commands
    eval(optcmds{ii});
  end;
  title(inputname(state+1));
  drawnow;                                   % flush graphics
  pause(1/Hz);
  state = ~state;
end;

% restore original properties
set(gcf,'DoubleBuffer',dborig);
