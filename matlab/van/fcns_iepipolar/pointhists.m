function pointhists(Pts,ii);
  
if ~exist('ii','var') || isempty(ii);
  ii = 1:length(Pts.gammaN);
end;

gbin = -0.1:0.001:0.1;
abbin = -20:0.001:40;

subplot(6,1,1); hist(Pts.gammaN(ii),gbin);  legend('gamma N');
subplot(6,1,2); hist(Pts.gammaS(ii),gbin);  legend('gamma S');
subplot(6,1,3); hist(Pts.alphaN(ii),abbin); legend('alpha N');
subplot(6,1,4); hist(Pts.alphaS(ii),abbin); legend('alpha S');
subplot(6,1,5); hist(Pts.betaN(ii),abbin);  legend('beta N');
subplot(6,1,6); hist(Pts.betaS(ii),abbin);  legend('beta S');
