% demo Covariance Intersection

Paa = rhomatrix(randcov(2));
Pbb = rhomatrix(randcov(2));

Pbb = 1.2*Pbb;

invPaa = spdinverse(Paa);
invPbb = spdinverse(Pbb);

figure(1); clf;
mu = [0;0];
draw_ellipse(mu,Paa,1,'r');
hold on;
draw_ellipse(mu,Pbb,1,'b');

for omega=linspace(0.05,0.95,20);
  invPcc = omega*invPaa + (1-omega)*invPbb;
  Pcc = spdinverse(invPcc);
  draw_ellipse(mu,Pcc,1,'-.','color',0.7*[1,1,1]);
end;

[Pcc_tr,omega_tr]   = covintersect(Paa,Pbb,'trace');
[Pcc_det,omega_det] = covintersect(Paa,Pbb,'det');

h(1) = draw_ellipse(mu,Pcc_tr,1,'m');
h(2) = draw_ellipse(mu,Pcc_det,1,'c');

hold off;

str = sprintf(['Covariance Intersection Demo\n', ...
	       '\\omega_{trace}=%.2f  \\omega_{det}=%.2f'], ...
	      omega_tr,omega_det);
title(str);
legend(h,'trace','det',-1)
axis equal tight;
