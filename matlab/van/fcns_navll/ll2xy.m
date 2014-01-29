function [x,y] = ll2xy(lat, lon, orglat, orglon)

x = (lon - orglon) .* mdeglon(orglat); 
y = (lat - orglat) .* mdeglat(orglat);
