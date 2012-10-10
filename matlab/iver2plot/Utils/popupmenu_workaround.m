function popupmenu_workaround(callback_handle)

if getappdata(gcbf, 'locked');
    %disp ('GUI locked, returning');
    return;
end

setappdata(gcbf, 'locked', true);
feval (callback_handle);
pause (0.1);
setappdata(gcbf, 'locked', false);
