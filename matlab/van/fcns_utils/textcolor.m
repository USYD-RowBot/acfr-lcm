function textcolor(arg_attr, arg_fg, arg_bg)
%TEXTCOLOR change UNIX terminal font color.
%   TEXTCOLOR(ATTRIBUTE,FG,BG) sets the terminals attribute, foreground
%   color, and background color to those specificed.  These settings remain
%   until the user resets them back.  The following attribute/colors can be
%   specified using either case independent strings, or numbers directly, or
%   combinations thereof.
%
%        RESET         0       BLACK         0
%        BRIGHT        1       RED           1
%        DIM           2       GREEN         2
%        UNDERLINE     3       YELLOW        3
%        BLINK         4       BLUE          4
%        REVERSE       7       MAGENTA       5
%        HIDDEN        8       CYAN          6
%                              WHITE         7
%
%   Example:
%   textcolor('bright','red','black'); % set termianl to brigh red on black
%   fprintf('In Color!\n');            % print on screen
%   textcolor('reset',7,0);            % reset terminal to white on black
%
%
%   Ryan M. Eustice 11-05-2004
%   Woods Hole Oceanographic Institution, MS 7
%   Woods Hole, MA 02543
%   508.289.3269   ryan@whoi.edu

% History
% DATE          WHO                      WHAT
%----------    ------------------        ------------------------------
%2004-11-05    Ryan Eustice              Created & written.

attr = parseAttribute(arg_attr);
fg   = parseColor(arg_fg);
bg   = parseColor(arg_bg);

fprintf('%c[%d;%d;%dm',hex2dec('1B'), attr, fg+30, bg+40);

%=============================================================
function attr = parseAttribute(arg_attr)
switch lower(arg_attr)
case {'reset',0},     attr = 0;
case {'bright',1},    attr = 1;
case {'dim',2},       attr = 2;
case {'underline',3}, attr = 3;
case {'blink',4},     attr = 4;
case {'reverse',7},   attr = 7;
case {'hidden',8},    attr = 8;
otherwise
 error('arg_attr didn''t match any known types');
end

%=============================================================
function color = parseColor(arg_color)
switch lower(arg_color)
case {'black',0},   color = 0;
case {'red',1},     color = 1;
case {'green',2},   color = 2;
case {'yellow',3},  color = 3;
case {'blue',4},    color = 4;
case {'magenta',5}, color = 5;
case {'cyan',6},    color = 6;
case {'white',7},   color = 7;
otherwise
 error('arg_color didn''t match any known types');
end
