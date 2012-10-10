function []=camfov()
% function calc_overlap based on camera fov
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2010-01-06      ak          Created from simple_gui2 (matlab fcn)
%    2010-01-07      ak          support two different FOV
%    2010-08-20      ak          Divided into ctrl win and plot win

%  Create and then hide the GUI as it is being constructed.
scrsz = get(0,'ScreenSize'); % dividing it by 2 (quarter size figure)

% to change design parameter, see init_range_fov()
user = init_range_fov;

ctrlbar_h = 100; user.ctrlbar_h = ctrlbar_h;
plt_w = scrsz(3)/2; plt_h = scrsz(4)/2 + ctrlbar_h;

figure(1);  clf;
set(1,'Name','camfov', ...
      'NumberTitle','off', ...
      'Units','pixels', ...
      'Position',[1 scrsz(4)/2 plt_w plt_h], ...
      'Resize','on', ...
      'Toolbar','none', ...
      'Tag','FIGPLOT', ...
      'UserData',user);

Control_Window (user);
