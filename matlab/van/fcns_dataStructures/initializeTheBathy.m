function bathy_t = initializeTheBathy(null);
%function bathy_t = initializeTheBathy(null);
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-20-2004      rme         Created and written.

% empty bathy_t data structure
bathy_t.X     = [];
bathy_t.Y     = [];
bathy_t.Z     = [];
bathy_t.alt   = [];
bathy_t.Cov_Z = [];
bathy_t.bnum  = [];
bathy_t.u     = [];
bathy_t.v     = [];
bathy_t.uc    = [];
bathy_t.vc    = [];
bathy_t.isel  = [];
bathy_t.altmin= [];
bathy_t.alt10 = [];
bathy_t.alt50 = [];
bathy_t.alt90 = [];
bathy_t.altmax= [];
 
