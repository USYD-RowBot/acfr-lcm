function SetTimeAxisFormat()

user = get(1,'UserData');

try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
end



figure(1);
ctlhdl = get(2,'UserData');
handle = get(1,'Children');
for ii = 1:length(handle)
    if strcmpi(get(handle(ii),'Tag'),'legend')
        continue;
    elseif strcmpi(get(handle(ii),'Type'),'axes')
        if (get(ctlhdl.long_radiobutton,'Value') == 1)
            %	set(handle(ii),'XTickLabelMode','Auto','XTickMode','Auto');
            Xshort = get(handle(ii),'XTick');
            Xlong = num2str(datestr(matlabtime(Xshort), 13));
            set(handle(ii),'XTickLabel',Xlong);
            axes(handle(ii));
            xlabel('Mission Time [HH:MM:SS]');
        elseif (get(ctlhdl.abs_radiobutton,'Value') == 1)
            %	set(handle(ii),'XTickLabelMode','Auto','XTickMode','Auto');
            Xshort = get(handle(ii),'XTick');
            Xlong = num2str(datestr(matlabtime(Xshort + user.STARTTIME), 13));
            set(handle(ii),'XTickLabel',Xlong);
            axes(handle(ii));
            xlabel('GMT Time [HH:MM:SS]');
        elseif (get(ctlhdl.short_radiobutton,'Value') == 1)
            set(handle(ii),'XTickLabelMode','Auto','XTickMode','Auto');
            axes(handle(ii));
            xlabel('Mission Time [s]');
        end
    end
end

