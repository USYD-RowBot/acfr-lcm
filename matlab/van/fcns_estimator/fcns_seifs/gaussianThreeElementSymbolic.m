clc;
clear all;

% allocate symbolic variables
syms mu_a mu_b mu_c sigma_a sigma_b sigma_c rho_ab rho_ac rho_bc real;

%=========================================================
% TRUE DISTRIBUTION      p(a,b,c)
%=========================================================
mu0    = [mu_a; mu_b; mu_c];
Sigma0 = [sigma_a^2,              rho_ab*sigma_a*sigma_b, rho_ac*sigma_a*sigma_c; ...
	  rho_ab*sigma_b*sigma_a, sigma_b^2,              rho_bc*sigma_b*sigma_c; ...
	  rho_ac*sigma_c*sigma_a, rho_bc*sigma_c*sigma_b, sigma_c^2];

%=========================================================
% APPROXIMATE DISTRIBUTION p_tilde(a,b,c)
%=========================================================
% projection matrices
S_ac = projection_matrix(3,[1,3]);
S_bc = projection_matrix(3,[2,3]);
S_c  = projection_matrix(3,3);

% calculate p_tilde(a,b,c)
mu0_tilde = mu0;
Lambda0_tilde =  S_ac * (S_ac'*Sigma0*S_ac)^-1 * S_ac' ...
               - S_c  * (S_c'*Sigma0*S_c)^-1   * S_c' ...
               + S_bc * (S_bc'*Sigma0*S_bc)^-1 * S_bc';
Lambda0_tilde = simple(Lambda0_tilde);
Sigma0_tilde = Lambda0_tilde^-1;
Sigma0_tilde = simple(Sigma0_tilde);

E0 = Sigma0_tilde - Sigma0;

%=========================================================
% COMPARE EFFECT OF MEASUREMENT UPDATE
%=========================================================
syms ha hb hc z sigma_z real;

% case 1: z = f(a,b)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% p(a,b,c)
H1 = [1 1 0];
[K1,mu1,Sigma1] = kupdate(mu0,Sigma0,H1,z,sigma_z);

% p_tilde(a,b,c)
[K1_tilde,mu1_tilde,Sigma1_tilde] = kupdate(mu0_tilde,Sigma0_tilde,H1,z,sigma_z);

E1 = simple(Sigma1_tilde - Sigma1);

% case 2: z = f(a,c)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% p(a,b,c)
H2 = [1 0 1];
[K2,mu2,Sigma2] = kupdate(mu0,Sigma0,H2,z,sigma_z);

% p_tilde(a,b,c)
[K2_tilde,mu2_tilde,Sigma2_tilde] = kupdate(mu0_tilde,Sigma0_tilde,H2,z,sigma_z);

E2 = simple(Sigma2_tilde - Sigma2);

% case 3: z = f(b,c)
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% p(a,b,c)
H3 = [0 1 1];
[K3,mu3,Sigma3] = kupdate(mu0,Sigma0,H3,z,sigma_z);

% p_tilde(a,b,c)
[K3_tilde,mu3_tilde,Sigma3_tilde] = kupdate(mu0_tilde,Sigma0_tilde,H3,z,sigma_z);

E3 = simple(Sigma3_tilde - Sigma3);
