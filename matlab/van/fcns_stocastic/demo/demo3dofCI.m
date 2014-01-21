% demo Covariance Intersection

n = 3;
Paa = rhomatrix(randcov(n));
Pbb = rhomatrix(randcov(n));

S = diag([1,1,1]);
Paa = S*Paa*S';
Pbb = S*Pbb*S';

invPaa = spdinverse(Paa);
invPbb = spdinverse(Pbb);

alpha = 1-2*normcdf(-1);
k2 = chi2inv(alpha,2);
k3 = chi2inv(alpha,3);

figure(1); clf;
mu = [0;0];
draw_ellipse(mu,Paa(1:2,1:2),k2,'r');
hold on;
draw_ellipse(mu,Pbb(1:2,1:2),k3,'b');

for omega=linspace(0.05,0.95,20);
  invPcc = omega*invPaa + (1-omega)*invPbb;
  Pcc = spdinverse(invPcc);
  draw_ellipse(mu,Pcc(1:2,1:2),k2,'-.','color',0.7*[1,1,1]);
end;

[Pcc_tr,omega_tr]   = covintersect(Paa,Pbb,'trace');
[Pcc_det,omega_det] = covintersect(Paa,Pbb,'det');

h(1) = draw_ellipse(mu,Pcc_tr(1:2,1:2),k2,'m');
h(2) = draw_ellipse(mu,Pcc_det(1:2,1:2),k2,'c');

hold off;

str = sprintf(['Covariance Intersection Demo\n', ...
	       '\\omega_{trace}=%.2f  \\omega_{det}=%.2f'], ...
	      omega_tr,omega_det);
title(str);
legend(h,'trace','det',-1)
axis equal tight;


figure(2); clf;
[Va,Da] = eig(Paa);
[Vb,Db] = eig(Pbb);
[Vc,Dc] = eig(Pcc_tr);
[X,Y,Z] = sphere(25);
X = X(:); Y = Y(:); Z = Z(:);

Ptsa = Va*sqrt(k3*Da)*[X';Y';Z'];
Ptsb = Vb*sqrt(k3*Db)*[X';Y';Z'];
Ptsc = Vc*sqrt(k3*Dc)*[X';Y';Z'];

N = length(X)^0.5;
[Xa,Ya,Za] = deal(Ptsa(1,:),Ptsa(2,:),Ptsa(3,:));
Xa = reshape(Xa,N,N);
Ya = reshape(Ya,N,N);
Za = reshape(Za,N,N);

[Xb,Yb,Zb] = deal(Ptsb(1,:),Ptsb(2,:),Ptsb(3,:));
Xb = reshape(Xb,N,N);
Yb = reshape(Yb,N,N);
Zb = reshape(Zb,N,N);

[Xc,Yc,Zc] = deal(Ptsc(1,:),Ptsc(2,:),Ptsc(3,:));
Xc = reshape(Xc,N,N);
Yc = reshape(Yc,N,N);
Zc = reshape(Zc,N,N);

cmap = jet;
Ca = ones(size(Za));
Cb = 32*Ca;
Cc = 64*Ca;
ha = surf(Xa,Ya,Za,Ca);
hold on;
hb = surf(Xb,Yb,Zb,Cb);
hc = surf(Xc,Yc,Zc,Cc);
hold off;

colormap(cmap);
set(ha,'facealpha',0.1);
set(hb,'facealpha',0.1);
set(hc,'facealpha',0.5);

shading flat;
lighting gouraud;
light;

[xa,ya] = calculateEllipseXY(mu,Paa(1:2,1:2),k2);
[xb,yb] = calculateEllipseXY(mu,Pbb(1:2,1:2),k2);
[xc,yc] = calculateEllipseXY(mu,Pcc(1:2,1:2),k2);

z = repmat(min([Za(:);Zb(:);Zc(:)]),size(xa))-2;

hold on;
plot3(xa,ya,z,'r-');
plot3(xb,yb,z,'b-');
plot3(xc,yc,z,'m-');
hold off;
