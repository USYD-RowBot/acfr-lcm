% script mccpselectcallback
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-16-2003      rme         Created and written.

if exist('input_points','var') && exist('base_points','var')
  % grab hidden figure handle where xform data is stored
  fig_hdl = findobj('Tag','manual_corr_cpselect');
  data = get(fig_hdl,'UserData');
  % close hidden figure
  delete(fig_hdl);
  
  % Warp the manually selected points back into the original image.
  % Note cpselect assumes coordinate system input and base coordinate
  % systems range from (1,1) to (nr,nc).  However, the warped images J1
  % and J2 have coordinate systems described by xdata1,ydata1 and
  % xdata2,ydata2 therefore we correct the cpselected points before
  % transforming them back into the original image space.
  input_points(:,1) = input_points(:,1) - (1-data.xdata1(1));
  input_points(:,2) = input_points(:,2) - (1-data.ydata1(1));
  tmp = tforminv(input_points,data.T_Hinf1);
  u1 = tmp(:,1); v1 = tmp(:,2);
  base_points(:,1) = base_points(:,1) - (1-data.xdata2(1));
  base_points(:,2) = base_points(:,2) - (1-data.ydata2(1));
  tmp = tforminv(base_points-1,data.T_Hinf2);
  u2 = tmp(:,1); v2 = tmp(:,2);

  % since the images we warped to common orientation, try to fine-tune
  % the user selected points using normalized cross-correlation
%  if size(data.J1,3) > 1; data.J1 = rgb2gray(data.J1); end
%  if size(data.J2,3) > 1; data.J2 = rgb2gray(data.J2); end
%  input_points_adj = cpcorr(input_points,base_points,data.J1,data.J2);
%  input_points_adj(:,1) = input_points_adj(:,1) - (1-data.xdata1(1));
%  input_points_adj(:,2) = input_points_adj(:,2) - (1-data.ydata1(1));
%  tmp = tforminv(input_points_adj,data.T_Hinf1);
%  u1adj = tmp(:,1); v1adj = tmp(:,2);
  
  % show correspondences
  figure(1); 
  imshow(data.udata,data.vdata,data.I1,'notruesize'); 
  title(sprintf('I1 Putative Matches: %d',length(u1)));
  hold on; 
  plot_multicolor(u1,v1,'+');
  plot_multicolor(u1,v1,'o');
  hold off;

  figure(2);
  imshow(data.udata,data.vdata,data.I2,'notruesize');
  title(sprintf('I2 Putative Matches: %d',length(u2)));
  hold on; 
  plot_multicolor(u2,v2,'+');
  plot_multicolor(u2,v2,'o');
  hold off;
end
