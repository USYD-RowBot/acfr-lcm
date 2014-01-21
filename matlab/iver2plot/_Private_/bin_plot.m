%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  [h] = bin_plot(data,nbins,pt_color,mn,mx,cmap_string) 
%
%  data.x data.y data.z, will be binned according to z 
%  data.scan, data.azimuth, data.range will be binned according to range
%  
%  nbins number of bins
%  
%  data may be a structure array. all levels of the array will be 
%  combined and then plotted
%
%  return h is the handle to the colorbar, use it to change the axis label.
%  default label is depth
% 
%  optional:
%  pt_color = a vector the same length as data.X that can be used for
%  the color coding. Nx1 vector
%  mn = range to associate with minimum color 
%  mx = range to associate with maximum color
%  cmap_string, like 'jet' or 'hot'
%
%  useage examples:
%
%  bin_plot(data,nbins,cmap_string) 
%  bin_plot(data,nbins,mn,mx,cmap_string) 
%  bin_plot(data,nbins,mn,mx) 
%  bin_plot(data,nbins,pt_color,mn,mx) 
%  bin_plot(data,nbins,pt_color) 
%
%  - needs bin_sort.m and axis labeling _mine functions
%
%  cnr - 01/10/2004 - create
%  cnr - 07-20-2004 - changed to handle structure arrays (uses squeeze_struct)
%  cnr - 1-23-2005  - added an addition input vector that can be
%                     used for coloring.
%  cnr - 3-13-2005  - fixed a max, min thing that caused errors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [h] = bin_plot(data,nbins,a,b,c,d)

%organize data

%can do this if since the dot_cloud stucture used to process 
%vehicle based data does not have an azimuth field
if(~isfield(data,'x'))
  fprintf(1,'Plotting angular space values\n');   
  data.x = data.scan;
  data.y = data.azimuth;    
  data.z = data.range;    
  range_image_plot = 1;
else
  range_image_plot = 0;  
end

[aa,dumb] = size(data.x);
if(aa == 1)
  data.x = data.x';
  data.y = data.y';
  data.z = data.z';
end  

if(length(data) > 1)      
  data = squeeze_struct(data);
end

%optional arugument parsing

if(nargin == 2)
  cmap_string = 'jet';
  color_vec = data.z;      
  mn = min( color_vec );
  mx = max( color_vec );
end

if(nargin == 3)
  if(isstr(a)) %color map given
    cmap_string = a;
    color_vec = data.z;    
    mn = min( color_vec );
    mx = max( color_vec );
  else
    color_vec = a;
    cmap_string = 'jet';
    mn = min( color_vec );
    mx = max( color_vec );
  end
end

if(nargin == 4)
  if(length(a) > 1)
    cmap_string = b;
    color_vec = a;
    mn = min( color_vec );
    mx = max( color_vec );
  else
    cmap_string = 'jet';
    color_vec = data.z;
    mn = a;
    mx = b;
  end
end

if(nargin == 5)
  if(length(a) > 1)
    cmap_string = 'jet';
    color_vec = a;
    mn = b;
    mx = c;
  else
    color_vec = data.z;
    mn = a;
    mx = b;
    cmap_string = c;    
  end
end

if(nargin == 6);
  cmap_string = d;  
  mn = b;
  mx = c;
  color_vec = a;  
end
    
%incase data is outside of mn and mx, trim it off
if(min(color_vec) < mn | max(color_vec) > mx)
  index = find(color_vec > mn & color_vec < mx);
  data.x = data.x(index);
  data.y = data.y(index);
  data.z = data.z(index);
  color_vec = color_vec(index);
  fprintf(['\n*** WARNING - Some data was outside your [%f,%f] interval and' ...
	   ' chopped off ***\n'],mn,mx);
end

%make the plot

eval(sprintf('colors = colormap(\''%s(%d)\'');',cmap_string,nbins));

[bins,index] = binsort(color_vec,linspace(mn,mx,nbins)',...
                           [1:1:length(color_vec)]');
   
for j = 1:length(index)
  plot3(data.x(bins{index(j)}(:,2)),data.y(bins{index(j)}(:,2)),...
	data.z(bins{index(j)}(:,2)),'.','Color',colors(index(j),:),...
	'LineWidth',4);
  hold on;
end
hold off

if(range_image_plot)
  xlabel_mine('Scan angle, (degrees)')  
  ylabel_mine('Azimuth angle, (degrees)')  
  zlabel_mine('Range, (meters)')
else    
  label_axes_xyz;
end

a = gca;

view([0,0,1]);
axis equal;

caxis([mn,mx]);
colormap(cmap_string)
h = colorbar;
axes(h);
caxis([mn,mx]);
ylabel_mine('Depth [m]')  

%set(h,'ylim',[mn,mx]);
%ch = get(h,'children');
%set(ch,'ydata',[mn, mx]);
  
axes(a);
  







