function varargout = hist_percentDT(TheJournal,Nrbins)
%function varargout = hist_percentDT(TheJournal,Nrbins)  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    05-06-2005      rme         Created and written.

if ~exist('Nrbins') || isempty(Nrbins); 
  Nrbins = 250; 
end;

Nf = TheJournal.Index.Nf;
Xp_i = TheJournal.Index.Xp_i;

% distance traveled
distance = TheJournal.Pathlen.pathvec2;

% delayed-state XY precision
sigma_d = zeros(Nf,1);
A = [1 -1];
for k=1:Nf;
  ii = TheJournal.Index.Xf_ii{k}(Xp_i(1:2));
  
  sigma_d(k) = sqrt(A*TheJournal.Eif.Sigma(ii,ii)*A');
end;

% percent distance traveled
pdt = sigma_d(2:end)./distance(2:end)*100; % avoid divide by zero on first state
					   % which has zero distance

% display histogram
[N,X] = hist(pdt,Nrbins);
h = bar(X,N,'b');
set(gca,'fontsize',12);
grid on;
pbaspect([3 1 1]);
axis tight;
xlabel('Percent Distance Traveled');
title('Histogram of Percent Distance Traveled for All States in the Pose-Network');

if nargout > 0;
  varargout{1} = h;
end;
