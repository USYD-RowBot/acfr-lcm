%close all
d = pwd;
dive=d(find( d == '_', 1, 'last')+1:end);

% interpolate the AUV_CONTROL depth reference onto the ACFR_NAV timebase
depth_ref = interp1(nav_t.AUV_CONTROL.utime, nav_t.AUV_CONTROL.depth, nav_t.ACFR_NAV.utime);
depth_kp = 0.25;
pitch_kp = 1.5;
depth_sat = 0.3;
pitch_sat = 0.3;
vel_sat = 1500;

pitch_ref = max(min((nav_t.ACFR_NAV.depth - depth_ref)*depth_kp, depth_sat),-depth_sat);
plane_ref = max(min((pitch_ref - nav_t.ACFR_NAV.pitch)*pitch_kp, pitch_sat), -pitch_sat);


figure(1)
clf

startI = 1;%find( nav_t.IVER_MOTOR.main > 0, 1, 'first' );
endI   = length(nav_t.ACFR_NAV.utime);%find( nav_t.IVER_MOTOR.main(startI+1:end) == 0, 1, 'first' );
fprintf( 'Mission duration %.2fsec\n', ...
    (nav_t.ACFR_NAV.utime(endI)-nav_t.ACFR_NAV.utime(startI))/1e6 )

xStart = (nav_t.ACFR_NAV.utime(startI)-nav_t.ACFR_NAV.utime(1))/1e6-1;
xEnd   = (nav_t.ACFR_NAV.utime(endI)-nav_t.ACFR_NAV.utime(1))/1e6+2+xStart;


% %%%%%%%%%%%%%% Velocities %%%%%%%%%%%%%%%%%%%%%
subplot(3,1,1)
hold on
title(dive)
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.ACFR_NAV.vx, 'b')
plot((nav_t.AUV_CONTROL.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.AUV_CONTROL.vx, 'r')
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.main/vel_sat, 'g')
legend('speed', 'speed ref', 'rpm', 'location', 'eastoutside');
grid on
xlim( [xStart xEnd] )
xlabel( 'time [s]' )
ylabel( 'vel [m/s]' )
[v i] = max(nav_t.ACFR_NAV.vx(startI:endI)); i = i+startI;
plot( (nav_t.ACFR_NAV.utime(i)-nav_t.ACFR_NAV.utime(1))/1e6, v, 'bo' )
fprintf( 'Des vel  =%.2f, Max vel  =%.2f after %.2fsec\n', ...
    max(nav_t.AUV_CONTROL.vx), v, (nav_t.ACFR_NAV.utime(i)-nav_t.ACFR_NAV.utime(1))/1e6 )

% Depth
subplot(3,1,2)
hold on
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, -nav_t.ACFR_NAV.depth, 'b')
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, -depth_ref, 'r')
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, -nav_t.ACFR_NAV.depth-nav_t.ACFR_NAV.altitude, 'k')
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, -nav_t.ACFR_NAV.depth-nav_t.ACFR_NAV.altitude.*cos(nav_t.ACFR_NAV.pitch), 'c')
legend('depth', 'depth ref', 'alt','location', 'eastoutside' )
grid on
xlim( [xStart xEnd] )
xlabel( 'time [s]' )
ylabel( 'depth [m]' )
[v i] = max(nav_t.ACFR_NAV.depth(startI:endI)); i = i+startI;
plot( (nav_t.ACFR_NAV.utime(i)-nav_t.ACFR_NAV.utime(1))/1e6, -v, 'bo' )
fprintf( 'Des depth=%.2f, Max depth=%.2f after %.2fsec\n', ...
    max(nav_t.AUV_CONTROL.depth), v, (nav_t.ACFR_NAV.utime(i)-nav_t.ACFR_NAV.utime(1))/1e6 )

% Pitch
subplot(3,1,3)
hold on
% actual and reference pitch
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.ACFR_NAV.pitch, 'b')
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, pitch_ref, 'r')
% actual and reference pitch
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.port, 'g')
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, plane_ref, 'c')
legend('pitch', 'pitch ref', 'plane', 'plane ref', 'location', 'eastoutside');
grid on
xlim( [xStart xEnd] )
xlabel( 'time [s]' )
ylabel( 'angle [1/s]' )

%% Roll
figure(2)
%subplot(5,2, plotI)
title(dive)
hold on
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.ACFR_NAV.roll, 'b')
% actual and reference pitch
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.port, 'g')
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.starboard, 'c')
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.top, 'r')
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.bottom, 'm')

legend('roll', 'port', 'starboard', 'top', 'bottom')

%%% Positions %%%%
figure(3)
hold on
scatter(nav_t.ACFR_NAV.longitude, nav_t.ACFR_NAV.latitude, 5, nav_t.ACFR_NAV.depth)
plot(nav_t.GPSD_CLIENT.longitude*180/pi, nav_t.GPSD_CLIENT.latitude*180/pi, '.r');

%%

% [x,y] = ll2xy(lat, lon, lat(startI), lon(startI));
% d = -nav_t.ACFR_NAV.depth;
% 
% figure(2);
% clf
% subplot(2,1,1)
% hold on
% plot3( x, y, d, '-' )
% plot3( 0, 0, d(startI), 'g*' )
% plot3( x(endI+startI), y(endI+startI), d(endI+startI), 'ro')
% grid on
% axis equal
% view(0,90)
% xlabel( 'x [m]' )
% ylabel( 'y [m]' )
% zlabel( 'z [m]' )
% 
% subplot(2,1,2)
% hold on
% plot3( x, y, d, '-' )
% plot3( 0, 0, d(startI), 'g*' )
% plot3( x(endI+startI), y(endI+startI), d(endI+startI), 'ro')
% grid on
% axis equal
% view(45,45)
% xlabel( 'x [m]' )
% ylabel( 'y [m]' )
% zlabel( 'z [m]' )
