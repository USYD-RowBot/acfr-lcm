% returns lat and lon in decimal degrees
% given x, y and the origin in lat, lon decimal degrees

function [lat,lon] = xy2ll(x, y, orglat, orglon)

lon = x./mdeglon(orglat) + orglon;
lat = y./mdeglat(orglat) + orglat;
