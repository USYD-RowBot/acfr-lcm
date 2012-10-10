function [bx,Hx] = marginalize_infoseq(bx,Hx,yi)
%MARGINALIZE_INFORMATION computes statistics of Gaussian marginal pdf.
%   [bx,Hx] = MARGINALIZE_INFORMATION(bz,Hz,yi) marginalizes the information
%   form of the jointly-Gaussian random variable Z over the elements
%   specified in the index vector yi using the specified information vector
%   bz and information matrix Hz.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-16-2004      rme         Created and written.

% The random vector z is composed of:
% z = [x', y']'
% Hz = [Hxx Hxy;
%       Hyx Hyy]
% and has a Gaussian distributed pdf of the form:
% p(z) = N^-1(bz,Hz)
%
% The marginal over x is given by:
% i.e.  p(x) = integral p(x,y) dy
%                 y

% The argument yi contains the indicies we wish to marginalize p(z) over

Ny = length(yi);
yi = reshape(sort(yi),Ny,1);

for ii=1:Ny
  % find marginal variable with most sparseness
  [Ne,kk] = min(sum(Hx(yi,:)~=0,2));
  
  % marginalize out yi(kk)
  jj = [1:length(Hx)];
  jj(yi(kk)) = [];
  bx = bx(jj) - bx(yi(kk))/Hx(yi(kk),yi(kk))*Hx(yi(kk),jj);
  Hx = Hx(jj,jj) - Hx(jj,yi(kk))*Hx(yi(kk),jj)/Hx(yi(kk),yi(kk));
  
  plot_corrcoef(Hx,1); drawnow;
  
  % update marginal variable index
  yi(kk+1:end) = yi(kk+1:end)-1;
  yi(kk) = [];
end
