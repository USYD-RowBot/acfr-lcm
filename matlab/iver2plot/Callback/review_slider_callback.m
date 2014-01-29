function review_slider_callback

ctlhdl = get(2,'UserData');
set(ctlhdl.setwin_checkbox,'Value',0);
value = get(ctlhdl.review_slider,'Value');
valprv = get(ctlhdl.review_slider,'UserData');
if (value > valprv);
    value = ceil(value);
else
    value = floor(value);
end

set(ctlhdl.review_slider,'Value',value);
set(ctlhdl.review_slider,'UserData',value);
if (value ~= -1);
    time_str = sprintf('Hour: %2d',value);
    set(ctlhdl.review_value_text,'String',time_str);
else
    set(ctlhdl.review_value_text,'String','Hour: ALL');
end

