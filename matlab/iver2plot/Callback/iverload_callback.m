function iverload_callback()

user   = get(1,'UserData');
ctlhdl = get(2,'UserData');

%===================================
% GRAB USER SELECTED .log FILE
%===================================
if 0==get(ctlhdl.uvclog_popupmenu,'Value')
    return;
else
    uvclog = popupstr(ctlhdl.uvclog_popupmenu);
    % check that the popupmenu is populated
    if (isempty(uvclog))
        errormsg();
        return;
    end
end

%addpath(genpath(pwd));
tmpname = [uvclog,'-iver_t.mat'];
tmpname_str = strrep(tmpname, '_', '\_');
if exist(tmpname,'file')
    h = waitbar(0.5, ['Cached version of ' tmpname_str ' exists.  Loading...']);
    load(tmpname,'-mat')
    waitbar(1, h, ['Loaded ', tmpname_str]);
    pause(0.5);
    close(h);
    assignin('base','iver_t',iver_t);
elseif exist(uvclog,'file')
    uvclog_waitbar_str = strrep(uvclog, '_', '\_');
    h = waitbar(0.5, ['Parsing ' uvclog_waitbar_str]);
    temp_t = iver2ParseToStruct(uvclog, user.UVC_VERSION);
    iver_t = temp_t.iver2;
    
    % save the workspace to the temp file
    waitbar(0.75, h, ['Saving ',tmpname_str]);
    save(tmpname, 'iver_t')
    waitbar(1, h, 'Loaded.');
    pause(0.5);
    close(h);
else
    errormsg();
    return;
end

%=============================================
% MAKE iver_t STRUCTURE AVAILABLE IN USER SPACE
%=============================================
assignin('base','iver_t',iver_t);


%==========================================================================
function errormsg()

errordlg('Loading UVC log file failed: correct directory?','Error');
