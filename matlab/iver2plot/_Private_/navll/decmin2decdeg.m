function [lat_decdeg,lon_decdeg] = decmin2decdeg(lat_decmin,lon_decmin)

lat_decdeg = fix(lat_decmin) + (lat_decmin-fix(lat_decmin))/60;
lon_decdeg = fix(lon_decdeg) + (lon_decmin-fix(lon_decmin))/60;
