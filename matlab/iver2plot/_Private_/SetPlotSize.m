function SetPlotSize;
  
h = get(gcf,'Children');
for i = 1:length(h)
    if strcmpi(get(h(i),'Tag'),'legend')
        position = get(h(i),'Position');
        position(1) = 0.87;
        set(h(i),'Position',position);
    elseif strcmpi(get(h(i),'Type'),'axes')
        position = get(h(i),'Position');

        %cnr- commented out for vertical color bar to work
        position(1) = 0.09;
        position(3) = 0.775;
        
        %position(2) = 0.1;
        %position(4) = 0.8;
        
        set(h(i),'Position',position);
    end
end
