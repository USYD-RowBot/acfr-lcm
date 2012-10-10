function dir_callback(flag)

ctlhdl = get(2,'UserData');

DIRECTORY = get(ctlhdl.dir_edittext,'String');
DIRECTORY = strrep(DIRECTORY,'~','/home/'); % expand "~" to fully qualified path name

% check that user entered a valid DIRECTORY name
ret = dir(DIRECTORY);
if (isempty(ret))
    errordlg('Not a valid DIRECTORY!','Error');
    return;
end
if DIRECTORY(end) == '/'
    DIRECTORY = DIRECTORY(1:end-1);
end
set(ctlhdl.dir_edittext,'UserData',DIRECTORY);
cd(DIRECTORY);

% finding *.log files
cmd_str = sprintf('%s/*.log',DIRECTORY);
uvc_logfiles = dir(cmd_str);
if (isempty(uvc_logfiles))
    set(ctlhdl.uvclog_popupmenu, ...
        'String','', ...
        'Value',0, ...
        'Min',1, ...
        'Max',1, ...
        'UserData',[]);
else
    uvc_loglist = uvc_logfiles(1).name;
    for ii = 2:length(uvc_logfiles)
        uvc_loglist = strcat(uvc_loglist,'|',uvc_logfiles(ii).name);
    end
    % show the list of log files for iver plot
    set(ctlhdl.uvclog_popupmenu, ...
        'String',uvc_loglist, ...
        'Value',1, ...
        'Min',1, ...
        'Max',length(uvc_logfiles), ...
        'UserData',strvcat(uvc_logfiles.name));
end

% finding lcmlog* files
lcmlog_cmd_str = sprintf('%s/lcmlog-*',DIRECTORY);
lcmlog_logfiles = dir(lcmlog_cmd_str);
if (isempty(lcmlog_logfiles))
    set(ctlhdl.lcmlog_popupmenu, ...
        'String','', ...
        'Value',0, ...
        'Min',1, ...
        'Max',1, ...
        'UserData',[]);
else
    lcm_loglist = lcmlog_logfiles(1).name;
    for ii = 2:length(lcmlog_logfiles)
        [pathstr,name,ext] = fileparts (lcmlog_logfiles(ii).name);
        if ~strcmp(ext,'.mat')
            lcm_loglist = strcat(lcm_loglist,'|',lcmlog_logfiles(ii).name);
        end
    end
    % show the list of log files for lcm plot
    set(ctlhdl.lcmlog_popupmenu, ...
        'String',lcm_loglist, ...
        'Value',1, ...
        'Min',1, ...
        'Max',length(lcmlog_logfiles), ...
        'UserData',strvcat(lcmlog_logfiles.name));
end
