function nav_t = loadjhu(navdir,navfile)
%LOADJHU  Loads a matlab nav_t.mat file.
%  NAV_T = LOADJHU(NAVDIR,NAVFILE) returns a NAV_T structure given the
%  data directory and the name of the .mat file.
%
%  EXAMPLE:
%  NAV_T = LOADJHU('/files1/data/jhu04/','nav_t.mat')
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-10-2004      rme         Created and written.
  
% load navigation data into nav data structure
tmp = load(strcat(navdir,navfile));
nav_t = tmp.nav_t;
