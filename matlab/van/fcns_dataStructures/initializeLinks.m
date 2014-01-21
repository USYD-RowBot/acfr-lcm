function Links = initializeLinks(TheConfig);
%function Links = initializeLinks(TheConfig);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-19-2004      rme         Moved out of process_init.m into a separete function
%    12-18-2004      rme         Renamed to initializeLinks.m

Links = struct('plinks',[],'vlinks',[],'fgraph',[],'xcorr_t',struct('seli',[],'selj',[]));

Links.plinks = spalloc(1,1,1);
Links.vlinks = spalloc(1,1,1);
Links.fgraph = spalloc(1,1,1);
