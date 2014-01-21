xtick = get(gca,'xtick');
clear xlab
for ii = 1:length(xtick)
   xlab(ii,:)=sprintf('%d',xtick(ii));
end
set(gca,'xticklabel',xlab);

ytick = get(gca,'ytick');
clear ylab
for ii = 1:length(ytick)
   ylab(ii,:)=sprintf('%d',ytick(ii));
end
set(gca,'yticklabel',ylab);
