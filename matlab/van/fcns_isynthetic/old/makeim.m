function [I1,I2] = makeim(x1,y1,x2,y2,idim,jdim);
%function [I1,I2] = makeim(x1,y1,x2,y2,idim,jdim);  
  
  xmax = max([x1(:); x2(:)])
  xmin = min([x1(:); x2(:)])
  
  ymax = max([y1(:); y2(:)])
  ymin = min([y1(:); y2(:)])
  
  sx = (jdim-1)/(xmax-xmin);
  sy = (idim-1)/(ymax-ymin);
  
  I1 = repmat(0,[idim jdim]);
  I2 = repmat(0,[idim jdim]);
  
  c1 = round(sx*(x1-xmin))*idim+round(sy*(y1-ymin))+1;
  c2 = round(sx*(x2-xmin))*idim+round(sy*(y2-ymin))+1;
  
  for k=1:length(x1)
    I1(c1(k))=1;
  end
  
  for k=1:length(x2)
    I2(c2(k))=1;
  end
