function botimg_publish (utime, filename, channel_name)
% function botimg_publish (utime, filename, channel_name)
% 
% DO:
% $ cmake. and then make before running this script
%
% matlab does have lcm interface but it takes long to copy
% from matlab image to bot_core_image_t data
% because there is no memcpy available.
% 
% this function calls a c executable that reads an image file and
% publishes to the channel provided.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2011-12-08      ak          Created.

% get full path name of executable
persistent pathexe pathstr;
if isempty(pathexe);
  pathstr = fileparts(which('botimg_publish.m'));
  pathexe = [pathstr,'/botimg_publish.sh'];
end;

cmd_line = sprintf ('%s %15.f %s %s', pathexe, utime, filename, channel_name);
curpath = pwd;
cd(pathstr);    
system(cmd_line);
cd (curpath);
