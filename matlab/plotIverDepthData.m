%close all
figure
subplot(2,1,1)
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, -nav_t.ACFR_NAV.depth)
hold on
plot((nav_t.AUV_CONTROL.utime-nav_t.ACFR_NAV.utime(1))/1e6, -nav_t.AUV_CONTROL.depth, 'r')
plot((nav_t.AUV_CONTROL.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.AUV_CONTROL.vx, 'm')
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.ACFR_NAV.vx, 'c')
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.main/1600, 'g')
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, -nav_t.ACFR_NAV.depth-nav_t.ACFR_NAV.altitude, 'k')
legend('depth', 'depth ref', 'speed ref', 'speed', 'rpm', 'alt');

grid on
depth_ref = max(nav_t.AUV_CONTROL.depth);
depth_kp = 0.2;
depth_sat = 0.125;
pitch_kp = 0.5;
pitch_sat = 0.125;
pitch_ref = max(min((nav_t.ACFR_NAV.depth-depth_ref)*depth_kp, depth_sat),-depth_sat);

subplot(2,1,2)
% actual pitch
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.ACFR_NAV.pitch)
hold on
%plot((nav_t.AUV_CONTROL.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.AUV_CONTROL.pitch, 'r')
%plot((nav_t.AUV_CONTROL.utime-nav_t.ACFR_NAV.utime(1))/1e6, -nav_t.AUV_CONTROL.depth*0.25, 'm')
% reference pitch
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, pitch_ref, 'r')

% pitch plane control signal
plot((nav_t.ACFR_NAV.utime-nav_t.ACFR_NAV.utime(1))/1e6, (pitch_ref - nav_t.ACFR_NAV.pitch)*pitch_kp, 'c')
plot((nav_t.IVER_MOTOR.utime-nav_t.ACFR_NAV.utime(1))/1e6, nav_t.IVER_MOTOR.port, 'g')
grid on
legend('pitch', 'pitch ref', 'plane ref', 'plane');