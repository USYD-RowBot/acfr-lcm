function SetZoom()

ctlhdl = get(2,'UserData');
handle = get(1,'Children');

figure(1);
if (get(ctlhdl.zoom_checkbox,'Value') == 1)
  drag off;
  zoom on;
else
  zoom off;
end
