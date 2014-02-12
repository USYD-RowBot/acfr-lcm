function clim = getclim

kids = get(1,'Children');
for ii=1:length(kids)
  if strcmp(get(kids(ii),'Tag'),'clim_inputbox')
    clim = str2num(get(kids(ii),'String'));
    break;
  end
end
