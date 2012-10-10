%=========================================================
% EVALUATE NUMERICALLY
%=========================================================
rand('state',sum(100*clock));
junk = { sigma_a, 1; ...
	 sigma_b, 1; ...
	 sigma_c, 1; ...
	 rho_ab,  0.6362; ...
	 rho_ac,  0.2101; ...
	 rho_bc, -0.1878 };
freevar = {junk{:,1}};
numeric = {junk{:,2}};

P0 = subs(Sigma0,freevar,numeric);
P0 = double(P0)
P0_tilde = subs(Sigma0_tilde,freevar,numeric);
P0_tilde = double(P0_tilde)

%=========================================================
% PLOT ELLIPSOIDS
%=========================================================
figure(1); clf;
[X,Y,Z] = sphere(25); 
X = X(:); Y = Y(:); Z = Z(:);
N = length(X)^0.5;
alpha = 1-2*normcdf(-3);
k3 = chi2inv(alpha,3);

% calculate point on ellipse boundary p(a,b,c)
[V0,D0] = eig(P0);
pts0 = V0*sqrt(k3*D0)*[X';Y';Z'];
[X0,Y0,Z0] = deal(pts0(1,:),pts0(2,:),pts0(3,:));
X0 = reshape(X0,N,N);
Y0 = reshape(Y0,N,N);
Z0 = reshape(Z0,N,N);

% calculate point on ellipse boundary p_tilde(a,b,c)
[V0_tilde,D0_tilde] = eig(P0_tilde);
pts0_tilde = V0_tilde*sqrt(k3*D0_tilde)*[X';Y';Z'];
[X0_tilde,Y0_tilde,Z0_tilde] = deal(pts0_tilde(1,:),pts0_tilde(2,:),pts0_tilde(3,:));
X0_tilde = reshape(X0_tilde,N,N);
Y0_tilde = reshape(Y0_tilde,N,N);
Z0_tilde = reshape(Z0_tilde,N,N);

% plot ellipsoids
cmap = jet(256);
C0 = zeros(size(Z0));
C0_tilde = C0+256;

h0 = surf(X0,Y0,Z0,C0);
hold on;
h0_tilde = surf(X0_tilde,Y0_tilde,Z0_tilde,C0_tilde);
hold off;

colormap(cmap);
set(h0,'facealpha',0.25);
set(h0_tilde,'facealpha',0.1);

%shading flat;
lighting gouraud;
light;
xlabel('a');
ylabel('b');
zlabel('c');
legend('p(a,b,c)','p\_tilde(a,b,c)');
