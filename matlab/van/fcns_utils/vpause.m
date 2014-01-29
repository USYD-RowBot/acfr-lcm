function varargout = vpause(t_wait,user_string,audible)
%VPAUSE variable length pause.
%   VPAUSE(T_WAIT,USER_STRING,AUDIBLE) any combination of options can be
%   specified, use an empty matrix [] to specify default values.  T_WAIT
%   is the amount of pause time in seconds, USER_STRING is a user defined
%   string for the message window, AUDIBLE is a binary flag specifying a
%   whether or not to use system beep to notify.
%
%   Main idea is to create an interactive pause; a cross between the PAUSE
%   and KEYBOARD functions.  PAUSE causes program execution to halt either
%   until the user presses a key or for a prespecified amount of time.  
%   KEYBOARD on the other hand causes the program to stop execution and
%   gives the user access to the current workspace.  VPAUSE is intended to
%   meld the functionality of these two operations.  VPAUSE will pause 
%   program execution until either:
%   (1) It times out and then resumes normal program flow.
%   OR
%   (2) A keyboard event occurs in which case a KEYBOARD command is issued
%       in the caller's workspace allowing the user to interactively view
%       the workspace.  Once the user is done, they can type RETURN to resume
%       normal program flow.
%
%   BOOLEAN = VPAUSE(T_WAIT,USER_STRING,AUDIBLE) returns a boolean value
%   so that the programer can control program execution.  BOOLEAN is true if
%   a keyboard event occured, false otherwise.
%
%   EX:
%   % wait up to 3 seconds for user input, otherwise continue.
%   vpause(3);
%
%   % wait up to 3 seconds, use the default title, and beep
%   vpause(3,[],1);
%
%   % print "hello world" if user input
%   if vpause(3); disp('hello world'); end;
%
%   See also KEYBOARD, DBQUIT
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-18-2003      rme         Created and written.
%    09-02-2004      rme         Updated to return immediately if t_wait <= 0
%    09-09-2004      rme         Figured out how to execute keyboard break
%                                in callers workspace from within vpause.m
%    11-02-2004      rme         Update print message.
%    12-06-2004      rme         Added audible beep option.
%    08-02-2006      rme         Added boolean output.

t_0 = clock;           % current time
pause_length = 0.025;  % seconds

% parse args
if ~exist('t_wait','var'); t_wait = inf; end;         % set default behavior to wait forever
if t_wait < pause_length;  ret = false;  return; end; % specified wait too short, return
if ~exist('user_string','var'); user_string = []; end;
if ~exist('audible','var') || isempty(audible); audible = false; end;

% setup a waitbar so that i can capture keyboard events.
data_t.event = false;
h = waitbar(0,'','Tag','kpause_waitbar','UserData',data_t);
set(h,'KeyPressFcn',@kpause_waitbar_callback);
  
% spin in this "while loop" until either we time out or get a keyboard event
t_countdown = t_wait;
tick = ceil(t_countdown)+1;
while ~data_t.event && (t_countdown > 0);
  pause(pause_length);
  t_countdown = t_wait-etime(clock,t_0);
  t_ceil = ceil(t_countdown);
  if tick - t_ceil > 0;     % check to see if a second has gone by
    tick = t_ceil;
    % update string
    if isempty(user_string);
      updated_title = sprintf(['Hit any key to enter keyboard mode,\n', ...
		    'automatic resume will occur in %d seconds.'],t_ceil);
    else;
      updated_title = user_string;
    end;
     % do a PC speaker beep
    if audible; beep; end;
  end;
  % update waitbar graphics
  waitbar(1-t_countdown/t_wait,h,updated_title);
  data_t = get(h,'UserData');
end;
close(h);

% check what to do depending on how we were called and whether or not
% the user interacted with the keyboard
if nargout; % the user requested boolean output
  varargout{1} = data_t.event;
elseif data_t.event; % the user desires keyboard interaction
  event = dbstack;
  fprintf('==>%s: event occured:\n%s at line %d\n',mfilename,event(end).name,event(end).line);
  evalin('caller','keyboard');
end;

% flush graphics buffer so Matlab closes waitbar window
drawnow;

%*****************************************************
function kpause_waitbar_callback(Ohandle,miscarg)
data_t = get(Ohandle,'UserData');
data_t.event = true;
set(Ohandle,'UserData',data_t);
