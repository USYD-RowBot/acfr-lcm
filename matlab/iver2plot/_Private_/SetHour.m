function SetHour()

ctlhdl = get(2,'UserData');
figure(1);

hour = get(ctlhdl.review_slider,'Value');
if (hour ~= -1)
  start = 3600 * hour;
  stop  = start + 3600;
  setwin([start,stop]);
else
  setwin('reset');
end
