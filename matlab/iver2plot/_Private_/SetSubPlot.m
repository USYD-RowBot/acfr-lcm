function [h,type] = SetSubPlot(user_defined)

if (nargin==0)
    ctlhdl = get(2,'UserData');
    figure(1);
    if (get(ctlhdl.top_radiobutton,'Value') == 1)
      h = subplot(2,1,1);
      type = 1;
      setfig_position;
    elseif (get(ctlhdl.bottom_radiobutton,'Value') == 1)
      h = subplot(2,1,2);
      type = 2;
      setfig_position;    
    elseif (get(ctlhdl.full_radiobutton,'Value') == 1)
      h = subplot(1,1,1);
      type = 0;
      setfig_position;    
    end
else
    % user_defined = 0, 1, 2
    % 0 = full, 1 = top, 2 = bottom
	ctlhdl = get(2,'UserData');
    figure(1);
    if (user_defined == 1)
      h = subplot(2,1,1);
      type = 1;
      setfig_position;
    elseif (user_defined == 2)
      h = subplot(2,1,2);
      type = 2;
      setfig_position;    
    elseif (user_defined == 0)
      h = subplot(1,1,1);
      type = 0;
      setfig_position;    
    end
end
  
function setfig_position()
position = get(gca,'Position');
position(1) = 0.09;
set(gca,'Position',position);



