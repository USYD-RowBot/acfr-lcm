function plot_corr_set(u1,v1,isel,ptype)
%PLOT_CORR_SET  Displays image pair correspondence set.
%  SAMPLE_CORR_SET(u1,v1,SEL,PTYPE) plots the image correspondence set.  
%  u1, v1 are length N vectors of hard correspondences.  SEL is an index of
%  inlier correspondences and PTYPE is a string indicating plot type,
%  'inlier', 'outlier', 'both' {default}.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    01-02-2003      rme         Created and written.
%    08-30-2003      rme         Added use of repmat to append NaNs
%    09-15-2003      rme         Renamed to plot_corr_set.m from
%                                sample_corr_set.m and rewrote to use
%                                plot_multicolor

if ~exist('sel','var')
  sel = [1:length(u1)];
end
if ~exist('ptype','var')
  ptype = 'both';
end

switch lower(ptype)
 case 'inlier', ptype = 1;
 case 'outier', ptupe = 2;
 case 'both',   ptype = [1 2];
end % switch lower(ptype)


% separate out outlier set
osel = setdiff([1:length(u1)],isel);

% plot inlier set & outlier sets
for i=1:length(ptype)
  switch ptype(i)
   case 1
    plot_multicolor(u1(isel),v1(isel),'+','linewidth',1.25);
    plot_multicolor(u1(isel),v1(isel),'o','linewidth',1.25);   
   case 2
    %plot_multicolor(u1(osel),v1(osel),'.','linewidth',1.25);
    plot_multicolor(u1(osel),v1(osel),'^','linewidth',1.25);  
   otherwise
    error('Unknown plot type');
  end % switch ptype(i)
end % for
