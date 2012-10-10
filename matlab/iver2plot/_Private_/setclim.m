function clim = setclim(clim);

kids = get(1,'Children');
for ii=1:length(kids)
  if strcmp(get(kids(ii),'Tag'),'clim_inputbox')
    set(kids(ii),'String',num2str(clim));
    break;
  end
end
