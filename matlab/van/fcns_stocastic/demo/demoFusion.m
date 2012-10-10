% demo Covariance Intersection

isnotPosDef = true;
while isnotPosDef;
  P    = rhomatrix(randcov(4));
  Paa  = P(1:2,1:2);
  Pab  = P(1:2,3:4);
  Pbb  = P(3:4,3:4);
  PPbb = 1.2*rhomatrix(randcov(2)); % conservative Pbb cov estimate
  [R,isnotPosDef] = chol(PPbb-Pbb);
end;

[Pbb_est,omega] = covfusion(Paa,Pab,PPbb,'det');

PP  = [Paa, Pab; Pab' PPbb];
Pest = [Paa, Pab; Pab' Pbb_est];

figure(1); clf;
mu = [0;0];
h(1) = draw_ellipse(mu,Paa,1,'k');
hold on;
h(2) = draw_ellipse(mu,Pbb,1,'b');
h(3) = draw_ellipse(mu,PPbb,1,'r');
h(4) = draw_ellipse(mu,Pbb_est,1,'m');

hold off;

str = sprintf(['Fusion Intersection Demo']);
title(str);
legend(h,'P_{aa}','P_{bb}','PP_{bb}','Estimate P_{bb}',-1)
axis equal tight;

[R1,p1] = chol(PP);
[R2,p2] = chol(Pest);
disp([p1,p2,omega]);
