function new_tform = make_2d_tformarray_tform(tform,in_size,udata,vdata,xdata,ydata)

out_size = [abs(ydata(2)-ydata(1))+1,abs(xdata(2)-xdata(1))+1];
out_size = round(out_size);
  
reg_b = maketform('box', fliplr(out_size(1:2)), ...
                  [xdata(1) ydata(1)], ...
                  [xdata(2) ydata(2)]);

in_size = in_size(1:2);

reg_a = maketform('box', fliplr(in_size), ...
                  [udata(1) vdata(1)], ...
                  [udata(2) vdata(2)]);

new_tform = maketform('composite', fliptform(reg_b), tform, reg_a);
%new_tform = maketform('composite', tform, reg_a);
