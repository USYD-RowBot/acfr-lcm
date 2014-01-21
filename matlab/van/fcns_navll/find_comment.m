figure(3)
clf
ind = [];
k=0;
while(1)
   ss = input('enter string to search for: ','s');

   for i = 1:n,
      if(~isempty(findstr(lower(ss),lower(p(i).comment))))
         k = k+1;
         ind(k)=i;
         fprintf('%.0f %.0f %s\n',xutm(i),yutm(i),p(i).comment);
      end
   end
   fprintf('found %d matches\n',k);


   axis([min(xutm(ind))-5,max(xutm(ind))+5,min(yutm(ind))-5,max(yutm(ind))+5]);
   if(k > 0)
      hold on
      for i = 1:k,
         j = ind(i);
         plot(xutm(j),yutm(j),[series_color(j),series_symbol(j)]);
         text(xutm(j),yutm(j),p(j).comment);
      end
   end
   sq
   grid on
   hold off
end
hold off
mylegend
   
      





