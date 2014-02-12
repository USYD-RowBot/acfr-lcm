function lcmload_callback()

user = get(1,'UserData');
ctlhdl = get(2,'UserData');

%===================================
% GRAB USER SELECTED lcmlog FILE
%===================================
if 0==get(ctlhdl.lcmlog_popupmenu,'Value')
    return;
else
    lcmlog = popupstr(ctlhdl.lcmlog_popupmenu);
    % check that the popupmenu is populated
    if (isempty(lcmlog))
        errormsg();
        return;
    end
end

%===================================
% LOAD IT
%===================================
tmpname = [lcmlog,'-nav_t.mat'];
tmpname_str = strrep(tmpname, '_', '\_');
if exist(tmpname,'file')
    h = waitbar(0.5, ['Cached version of ' tmpname_str ' exists.  Loading...']);
    load(tmpname)
    waitbar(1, h, ['Loaded ', tmpname_str]);
    pause(0.5);
    close(h);
    assignin('base','nav_t',nav_t);
elseif exist(lcmlog,'file')
    h = waitbar(0.5, ['Parsing ' lcmlog]);
    nav_t = lcmlog_export(lcmlog);
    nav_t = elapsed_time(nav_t);
    
    % save the workspace to the temp file
    waitbar(0.75, h, ['Saving ', tmpname_str]);
    save(tmpname, 'nav_t')
    waitbar(1, h, 'Loaded.');
    pause(0.5);
    close(h);
else
    errormsg();
    return;
end

%===================================
% post process RDI (Bad beam == -32.768)
% see perls/src/rdi.h
%===================================
if isfield (nav_t, 'RDI')
    rmidx = find(nav_t.RDI.btv(:) == -32.768);
    nav_t.RDI.btv(rmidx) = nan;
        
    rmidx = find(nav_t.RDI.wtv(:) == -32.768);
    nav_t.RDI.wtv(rmidx) = nan;
end

%=============================================
% MAKE NAV_T STRUCTURE AVAILABLE IN USER SPACE
%=============================================
assignin('base','nav_t',nav_t);



%==========================================================================
function errormsg()

errordlg('Loading loglog file failed: correct directory?','Error');

%==========================================================================
function nav_t = elapsed_time(nav_t)

%define starttime and endtime
STARTTIME = inf;
ENDTIME = -inf;
sensors=fieldnames(nav_t);
for s=1:length(sensors)
    sensor = sensors{s};
    % JMW hack
    try
        ts = nav_t.(sensor).utime(1);
        te = nav_t.(sensor).utime(end);
    
        if ts < STARTTIME
            STARTTIME = ts;
        end
        
        if te > ENDTIME
            ENDTIME = te;
        end
    catch
        fprintf ('ERROR %s\n', sensor);
        continue;
    end
end
STARTTIME = STARTTIME * 1e-6;
ENDTIME = ENDTIME * 1e-6;

%generate elapsed time
for s=1:length(sensors)
    sensor = sensors{s};
    
    % rename utime to unixtime
    utime = nav_t.(sensor).utime;
    nav_t.(sensor) = rmfield(nav_t.(sensor), 'utime');
   
    fields = fieldnames(nav_t.(sensor));
    nav_t.(sensor).unixtime = utime * 1e-6;
    nav_t.(sensor).elaptime = nav_t.(sensor).unixtime - STARTTIME;
    nav_t.(sensor) = orderfields(nav_t.(sensor), {'unixtime','elaptime', fields{:}});
end
nav_t.STARTTIME = STARTTIME;
nav_t.ENDTIME = ENDTIME;
