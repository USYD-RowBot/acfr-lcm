function SetDrag()
  
ctlhdl = get(2,'UserData');
handle = get(1,'Children');

figure(1);
if (get(ctlhdl.grid_checkbox,'Value') == 1)
  zoom off;
  drag on;
else
  drag off;
end
