function goal_t = calc_grid(startpos,heading,length,width,numlines,speed,param_t)
%CALC_GRID automatically calculate goal behaviors for a grid pattern.
%   GOAL_T = CALC_GRID(STARTPOS,HEADING,LENGTH,WIDTH,NUMLINES,SPEED,PARAM_T)
%   INPUTS:
%   STARTPOS: [2 x 1] vector of x,y start position in local-level coordinates
%   HEADING:  0 is North and grid is defined right to left.
%   LENGTH: length of grid
%   WIDTH:  inter-track spacing
%   NUMLINES: number of tracklines
%   SPEED:  XY speed [m/s]
%   PARAM_T: parameter structure associated with a line behavior
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-20-2003      rme         Created and written.

mode_type_t = mode_type_struct;
endpos = startpos;
for ii=1:numlines
  % LEG
  alt = (-1)^(ii-1);
  if alt == 1
    hdg = heading;
  else
    hdg = heading+pi;
  end
  endpos = startpos + length*[cos(hdg), sin(hdg)];
  goal_t(ii).XYMode = mode_type_t.XY_LINE;
  goal_t(ii).param  = param_t;
  goal_t(ii).param.start = startpos;
  goal_t(ii).param.stop  = endpos;
  goal_t(ii).param.speed = speed;
  % calculate start position of next leg
  startpos = endpos + width*[cos(hdg-alt*90*pi/180), sin(hdg-alt*90*pi/180)];
end
