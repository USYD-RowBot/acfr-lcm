function Ipolar = zernikePolarSample(Ipatch,Zbasis);
%function Ipolar = zernikePolarSample(Ipatch,Zbasis);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-15-2004      rme         Created and written.  

w = (length(Ipatch)-1)/2;

% sample points in input image space
co = (w+1); % center pixel of square image
[usamp,vsamp] = deal(Zbasis.xsamp*w + co, -Zbasis.ysamp*w + co);

% create a resampler structure
R = makeresampler('linear','fill');

% sample the raw image directly by specifying the raw image space sample points
% with a tmap_b data structure used within the tformarray command
tmap_b = cat(3,usamp,vsamp);
Ipolar = tformarray(Ipatch,[],R,[2 1],[1 2],[],tmap_b,0);
