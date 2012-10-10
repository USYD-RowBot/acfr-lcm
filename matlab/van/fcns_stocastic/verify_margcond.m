N = 9;
u = randn(N,1);
P = randcov(N);
H = P^-1;
b = H*u;

% p(a,b|c)
c = randn(3,1);
[u1,P1] = condgauss_cov(u,P,1:3,c);
[b1,H1] = condgauss_info(b,H,1:3,c);

% p(a|c)
[u2,P2] = marggauss_cov(u1,P1,1:3)
[b2,H2] = marggauss_info(b1,H1,1:3)

% p(a|c)
[u3,P3] = margcondgauss_cov(u,P,7:9,4:6,1:3,c)
[b3,H3] = margcondgauss_info(b,H,7:9,4:6,1:3,c)
