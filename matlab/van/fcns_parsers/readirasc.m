function [u1,v1,u2,v2,iname1,iname2] = readirasc(filename);
%function [u1,v1,u2,v2,iname1,iname2] = readirasc(filename);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2006-01-17      rme         Created and written.
%    2006-01-22      rme         Made parsing of layer name more robust.

fid = fopen(filename,'r');

% pre-allocate
iname1 = '';
iname2 = '';
u1 = zeros(250,1);
v1 = zeros(250,1);
u2 = zeros(250,1);
v2 = zeros(250,1);

% grab data
ii = 1;
while 1;
  tline = fgetl(fid);
  if tline == -1; break; end; % EOF
  
  if strncmpi(tline,'Control Layer Name:',19);
    iname1 = strread(tline,'Control Layer Name: %s',1);
  elseif strncmpi(tline,'Control x,y (pix):',18);
    [u1(ii),v1(ii)] = strread(tline,'Control x,y (pix): %f %f');
  elseif strncmp(tline,'Input Layer Name:',17);
    iname2 = strread(tline,'Input Layer Name: %s',1);
  elseif strncmpi(tline,'Input x,y (pix):',16);
    [u2(ii),v2(ii)] = strread(tline,'Input x,y (pix): %f %f');
    ii = ii+1;
  end;
  
end;

% discard excess
ii = ii-1;
u1 = u1(1:ii);
v1 = v1(1:ii);
u2 = u2(1:ii);
v2 = v2(1:ii);
