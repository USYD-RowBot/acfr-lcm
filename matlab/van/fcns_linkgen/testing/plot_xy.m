function plot_xy(mu,index_t);

xy_i = index_t.xyz_i(1:2);

% plot robot
Xv_i = index_t.Xv_i;
plot(mu(Xv_i(xy_i(2))),mu(Xv_i(xy_i(1))),'r*');

% plot delayed states
Nv = index_t.Nv;
Nf = index_t.Nf;
x_i = index_t.Xf_i(1:Nv:Nv*Nf);
y_i = index_t.Xf_i(2:Nv:Nv*Nf);
hold on;
plot_multicolor(mu(y_i),mu(x_i));
hold off;

axis equal;
grid on;


