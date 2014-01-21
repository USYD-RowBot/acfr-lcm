function plthdl = subplot_handle()

plthdl = [];
try
  kids = get(1,'Children');
catch
  return;
end

for ii=1:length(kids)
  if  strcmp(get(kids(ii),'Tag'),'subplot_frame') || strcmp(get(kids(ii),'Tag'),'subplot_frame_position')
   plthdl = get(kids(ii),'UserData');
    break;
  end
end
