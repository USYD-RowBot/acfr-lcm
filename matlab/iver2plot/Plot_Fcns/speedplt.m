function speedplt()
%speed plot

user = get(1,'UserData');

cla reset;
KN2MPS = 0.514444444;    % knots to m/s
legend_str='clea'; 

hold on;

% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
end

if IVER
    gps_speed = iver_t.gps.Speed_Knots*KN2MPS;
    gps_speed(iver_t.gps.Num_Sats == 0) = NaN;
    
    plot(iver_t.elaptime, gps_speed, 'c',...
         iver_t.elaptime, iver_t.rdi_Cur_Speed*KN2MPS, 'k','LineWidth',2);
    legend_str=strvcat(legend_str,'gps (uvc)','uvc');
    ylabel('m/s');
end

% lcmlog
try
    nav_t = evalin('base','nav_t');
    NAV=1;
catch
    NAV=0;
end

if NAV
    % x_vs = [0 0 0 180 0 135];
    R_vs = [-0.7071    0.7071    0.0000
             0.7071    0.7071    0.0000
             0.0000    0.0000   -1.0000];
    vel_veh = R_vs*nav_t.RDI.btv(:,1:3)';
    
    plot(nav_t.RDI.elaptime,vel_veh,'LineWidth',2);
    legend_str=strvcat(legend_str,'u (lcm)','v (lcm)','w (lcm)');
    ylabel('m/s');
end

hold off;
title('Speed (GPS & est)');
legend_str(1,:)=[]; legend(legend_str);
