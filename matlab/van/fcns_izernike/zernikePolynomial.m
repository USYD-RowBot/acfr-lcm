function V_nm = zernikePolynomial(n,m,rho,theta);
%Zernike Polynomial
%    V_nm = zernikePolynomial(n,m,rho,theta) evaluates the Zernike polynomial V_nm
%    for the given order n and repetition m over points within the unit circle
%    defined by polar coordinates rho and theta.  This implementation should be
%    accurate up to order n=43.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-09-2004      rme         Created and written.
%    12-10-2004      rme         Rewrote to make factorial calculation more accurate.

% loop over the summation index s to calculate the radial component
% R_nm as a function of rho
R_nm = zeros(size(rho));  
for s=0:(n-abs(m))/2;
  % naive implementation which is inaccurate for n>21 due to factorial limitations
  %-----------------------------------------------------------------------
  %num  = (-1)^s * factorial(n-s);
  %den  = factorial(s) * factorial((n+abs(m))/2-s) * factorial((n-abs(m))/2-s);
  %R_nm = R_nm + num/den*rho.^(n-2*s);

  % ****************************** note ***********************************
  % take extra care to handle calculation of factorials which can be a very
  % large number for even modest orders of n.
  % FACTORIAL(N) is only accurate for N <= 21, see matlab help file.
  N = n-s;
  D = [s, (n+abs(m))/2-s, (n-abs(m))/2-s];
  [Dmax,ind] = max(D);
  
  num = (-1)^s * prod(Dmax+1:N);
  den = D; den(ind) = [];
  if any(den > 21);
    warning('Result not accurate for orders greater than n=43');
  end;
  den = factorial(den(1)) * factorial(den(2));
  
  % evaluate radial function
  R_nm = R_nm + num/den*rho.^(n-2*s);
end;

% complex Zernike basis function evaluated at all (rho,theta)
V_nm = R_nm .* exp(j*m*theta);
