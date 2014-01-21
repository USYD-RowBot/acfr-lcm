%H = sprandsym(600,0.2,rand(600,1));
N = 300;
H = 10^0*sprandcov(N,0.05);
P = full(H^-1);
u = randn(N,1);
b = u'*H;

xij = [10:50:N]';
xij_plus = markov_blanket(H,xij)';
xij_minus = [1:N]';
xij_minus([xij;xij_plus]) = [];

dummy = randn(size(xij_plus));
dummy2 = randn(size(xij_minus));

%p(xi,xj | xij_plus)
[b1,H1] = margcondgauss_info(b,H,xij,xij_minus,xij_plus,dummy);

%p(xij_minus | xij_plus)
[b2,H2] = margcondgauss_info(b,H,xij_minus,xij,xij_plus,dummy);

%p(xi,xj,xij_minus | xij_plus)
ii = [xij_plus; xij; xij_minus];
[b3,H3] = condgauss_info(b(ii),H(ii,ii),1:length(xij_plus),dummy);

%p(xi,xj | xij_plus,xij_minus)
ii = [xij_plus; xij_minus; xij];
k  = length(xij_plus)+length(xij_minus);
[b4,H4] = condgauss_info(b(ii),H(ii,ii),1:k,[dummy; dummy2]);
[u4,P4] = condgauss_cov(u(ii),P(ii,ii),1:k,[dummy; dummy2]);
% the above expression shows that b1==b4 and H1==H4, therefore
% xi,xj are conditionally independent of xij_minus given xij_plus

% projection matricies
Sxij = projection_matrix(N,xij);
Sxij_minus = projection_matrix(N,xij_minus);
Sxijxij_minus = projection_matrix(N,[xij;xij_minus]);

% check conditional independence
%---------------------------------------------
% reorder terms to be consistent with a state vector
% of X = [xij,xij_minus]
b3_tilda = b3*Sxijxij_minus';
H3_tilda = Sxijxij_minus*H3*Sxijxij_minus';

% if xi,xj is conditionally independent of xij_minus given xij_plus
% then the following must be true:
% p(xi,xj,xij_minus | xij_plus) = p(xi,xj | xij_plus) * p(xij_minus | xij_plus)
b3_star = b1*Sxij' + b2*Sxij_minus';
H3_star = Sxij*H1*Sxij' + Sxij_minus*H2*Sxij_minus';
% test if the above condition holds true
if all(b3_star==b3_tilda) && all(all(H3_star==H3_tilda))
  disp('Conditionally independent: True');
else
  disp('Conditionally independent: False');
end

% another way to prove conditional independence is to look at the 
% sparsity pattern for P3 = H3^-1;  The blocks between xij and xij_minus
% are zero which means independence for a Gaussian distribution.
spy(H3^-1);
