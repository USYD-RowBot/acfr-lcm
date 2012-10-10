function mindex_t = incrementPxfMeasurementIndex(nav_t,mindex_t,TheConfig)
%function mindex_t = incrementPxfMeasurementIndex(nav_t,mindex_t,TheConfig)  
%
%  Facilitates the bookkeeping associated with the camera measurement
%  index based upon the predefined image sequence according to the config
%  file.
%
% HISTORY      WHO     WHAT
%----------    ----    -------------------------------------
% 04-08-2004   rme     Created and written.
% 04-12-2004   rme     Changed to only increment the PXF mindex,
%                      initalization is taken care of by mindex_t_struct.m
% 10-27-2004   rme     Renamed to incrementPxfMeasurementIndex.m from incpxfmindex.m
% 10-28-2004   rme     Major code reorganization.
% 01-03-2004   rme     Don't set mindex_t.PXF to -1.

% find current index within TheConfig image sequence
ii = find(nav_t.PXF.imgnum(mindex_t.PXF) == TheConfig.Data.imageSequence);
% increment the index within the TheConfig image sequence
ii = ii+1;

% return the PXF mindex associated with the next image
if ii <= length(TheConfig.Data.imageSequence)
  mindex_t.PXF = find(nav_t.PXF.imgnum == TheConfig.Data.imageSequence(ii));
else;
  mindex_t.PXF = mindex_t.PXF+1;
end;
