function varargout = plot_corrcoef(P,absflag)
%PLOT_CORRMCOEF plots correlation coefficient matrix.
%function h = plot_corrcoef(P,absflag)  
%  P correlation matrix
%  fignum  figure number
%  absflag (opt) true plots absolute value of correlation coeff (gray scale)
%                false plots correlation coeff in range [-1 1]  (jet scale)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-10-2004      rme         Created and written.
%    03-22-2004      rme         Renamed from plot_corrmat.m to plot_corrcoef.m
%    03-30-2004      rme         Compartmentalized and moved axis
%                                labeling to a separate function label_corrcoef.m
%    04-14-2004      rme         Got rid of fignum flag.
%    06-19-2004      rme         Switched pixel centers back to Matlab default.
%    06-24-2004      rme         Added nargout check for plot handle.
  
if ~exist('absflag','var') || isempty(absflag), absflag=false; end  

rho = rhomatrix(P);

if absflag
  h = imagesc([1,size(rho,1)]+.5,[1,size(rho,2)]+.5,abs(rho),[0 1]);
  %h = imagesc([1,size(rho,1)],[1,size(rho,2)],abs(rho),[0 1]);
  colormap(flipud(gray));
else
  h = imagesc([1,size(rho,1)]+.5,[1,size(rho,2)]+.5,rho,[-1 1]);
  %h = imagesc([1,size(rho,1)],[1,size(rho,2)],rho,[-1 1]);  
  colormap jet;
end
axis square;
colorbar('horiz');

if nargout
  varargout{1} = h;
end
