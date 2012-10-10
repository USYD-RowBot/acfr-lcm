function gobutton_callback()

user = get(1,'UserData');
ctlhdl = get(2,'UserData');

fprintf('\n');

if 0==get(ctlhdl.uvclog_popupmenu,'Value') && 0==get(ctlhdl.lcmlog_popupmenu,'Value')
   errordlg('This directory does not contain any UVC or LCM log files to review!','Error');
   return;
end

evalin('base','clear iver_t nav_t;');
popupmenu_workaround(@iverload_callback);
try
    iver_t = evalin('base','iver_t');
    IVER = 1;
catch
    IVER = 0;
end

popupmenu_workaround(@lcmload_callback);
try
    nav_t = evalin('base','nav_t');
    NAV = 1;
catch
    NAV = 0;
end

user.STARTTIME=0;
user.ENDTIME=0;

if NAV && IVER
    disp('reconciling nav_t and iver_t STARTTIME and ENDTIME');
    
    % UVC time designates start of mission, so nav_t elaptime should be
    % negative during the time *before* the mission starts
    user.STARTTIME = iver_t.STARTTIME;
    user.ENDTIME = iver_t.ENDTIME;
    
    dt = nav_t.STARTTIME - iver_t.STARTTIME;
    
    sensors = fieldnames(nav_t);
    for s=1:length(sensors)
        sensor = sensors{s};
        if strcmp (sensor,'STARTTIME') || strcmp (sensor,'ENDTIME')
            continue;
        end
        nav_t.(sensor).elaptime = nav_t.(sensor).elaptime + dt;
    end
    
    % this is a hack so that iver_t.elaptime spans the same timebase as
    % nav_t.elaptime (which should be longer b/c it starts logging before
    % UVC)
    iver_t.elaptime(1) = dt;
    iver_t.elaptime(end) = nav_t.ENDTIME - nav_t.STARTTIME + dt;
    
    nav_t.STARTTIME = iver_t.STARTTIME;
    nav_t.ENDTIME = iver_t.ENDTIME;

    % dvlrenav
    %att_src = 'OS_COMPASS';
    att_src = 'MICROSTRAIN';
    switch att_src
        case 'OS_COMPASS'
            x_vs_att = [0,0,0,[0,0,0]*DTOR]';
            x_lr_att = [0,0,0,[0,0,0]*DTOR]';
            att_rph = nav_t.OS_COMPASS.rph;
            att_unixtime = nav_t.OS_COMPASS.unixtime;
        case 'MICROSTRAIN'
            x_vs_att = [0,0,0,[0,0,180]*DTOR]';
            x_lr_att = [0,0,0,[0,0,0]*DTOR]';
            att_rph = nav_t.MICROSTRAIN.Euler;
            att_unixtime = nav_t.MICROSTRAIN.unixtime;
    end
    fprintf ('dvlrenav using %s\n', att_src);
    x_vs_dvl = [0,0,0,[180,0,135]*DTOR]';
    nav_t.DVLRENAV = dvlrenav (nav_t.RDI.unixtime, nav_t.RDI.btv, nav_t.RDI.btv_status, x_vs_dvl, ...
                               att_unixtime, att_rph, x_vs_att, x_lr_att, ...
                               [0,0,0], iver_t.gps.Magnetic_Variation(1)*DTOR);
    % correct origin w.r.t. iver
    [foo,ii] = min(abs(nav_t.DVLRENAV.t_abs-iver_t.unixtime(1)));
    xo = iver_t.x(1) - nav_t.DVLRENAV.nx(ii);
    yo = iver_t.y(1) - nav_t.DVLRENAV.ny(ii);
    zo = iver_t.z(1) - nav_t.DVLRENAV.nz(ii);
    nav_t.DVLRENAV.nx = nav_t.DVLRENAV.nx + xo; % xo w.r.t. iver's origin
    nav_t.DVLRENAV.ny = nav_t.DVLRENAV.ny + yo; % yo w.r.t. iver's origin
    nav_t.DVLRENAV.nz = nav_t.DVLRENAV.nz + zo; % zo w.r.t. iver's origin
    
                           
    assignin('base','iver_t',iver_t);
    assignin('base','nav_t',nav_t);
elseif IVER
    user.STARTTIME = iver_t.STARTTIME;
    user.ENDTIME = iver_t.ENDTIME;
elseif NAV
    user.STARTTIME = nav_t.STARTTIME;
    user.ENDTIME = nav_t.ENDTIME;
else
    errordlg('No log files to load!','Error');
end

%===================================
% DISPLAY MISSION BEGIN/END TIME
%===================================
text_str = sprintf('Start: %s',datestr(matlabtime(user.STARTTIME),0));
set(ctlhdl.stats_start_text,'String',text_str);
text_str = sprintf('End:  %s',datestr(matlabtime(user.ENDTIME),0));
set(ctlhdl.stats_end_text,'String',text_str);
text_str = sprintf('Elapsed Time: %s',datestr(matlabtime(user.ENDTIME-user.STARTTIME),13));
set(ctlhdl.stats_elapsed_text,'String',text_str);

set(1,'UserData',user);
