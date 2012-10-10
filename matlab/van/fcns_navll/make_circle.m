%guaymas
org_lat = 27 +  00.000/60;
org_lon= -111 -26.000/60;

fprintf('origin is: %.0f %.4f %.0f %.4f\n', ...
      floor(org_lat),60*(org_lat-floor(org_lat)), ...
      -floor(-org_lon),-60*(-org_lon-floor(-org_lon)))

% xpndr A
lat_ctr = 27 + 00.45/60;
lon_ctr = -111 - 25.29/60;

% xpndr B                  
%lat_ctr = 26 + 59.71/60;  
%lon_ctr = -111 - 24.38/60;

% xpndr C                  
%lat_ctr = 27 + 00.72/60;  
%lon_ctr = -111 - 23.88/60;



% xpndr D                  
%lat_ctr = 27 + 01.42/60;  
%lon_ctr = -111 - 24.82/60;

[Xcenter,Ycenter] = ll2xy(lat_ctr,lon_ctr,org_lat,org_lon); 
RADIUS = 1000;        
n = 100;

fid = fopen('backgrnd.dat','wt'); 
x = zeros(n,1);
y = zeros(n,1);
xutm = zeros(n,1);
yutm = zeros(n,1);
theta = 2*pi*[0:n-1]'/n;
x = Xcenter + RADIUS*sin(theta);
y = Ycenter + RADIUS*cos(theta);
for ii = 1:n,
   [lat,lon] = xy2ll(x(ii),y(ii),org_lat,org_lon);
   [xutm(ii),yutm(ii)] = ll2utm(lat,lon,12);
   fprintf(fid,'%.1f %.1f\n',xutm(ii),yutm(ii));
end
plot(xutm,yutm);
sq
fclose(fid);
