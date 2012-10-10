function I = generate_image(x,y,xo,yo,nc,nr);
%function I = generate_image(x,y,xo,yo,nc,nr);  
  I = repmat(0,[nr nc]);
  
  xi = round(x)+(nc/2-xo);
  yi = round(y)+(nr/2-yo);
  
  ind = (xi >=1 & xi <= nc) & (yi >=1 & yi <= nr);
  xi = xi(ind);
 
  yi = yi(ind);
  
  I((xi-1)*nr+yi) = 1;
