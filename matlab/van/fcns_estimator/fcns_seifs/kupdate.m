function [K,mu,Sigma] = kupdate(mu,Sigma,H,z,sigma_z);

K = Sigma*H'*(H*Sigma*H'+sigma_z^2)^-1;
K = simple(K);

mu = mu + K*(z-H*mu);
mu = simple(mu);

I = speye(size(Sigma));
Sigma = (I-K*H)*Sigma;
Sigma = simple(Sigma);
