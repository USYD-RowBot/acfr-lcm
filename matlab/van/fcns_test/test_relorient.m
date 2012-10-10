%test_relorient.m

if ~exist('rerun','var');
  rerun = false;
end

if rerun == false
% camera calibration matrix
nr = 1024;
nc = 1280;
po = (nc-1)/2;
qo = (nr-1)/2;
K = Kcam(1600,po,qo);

% random continuous surface
Nf = 1000;
xdim = 10;
ydim = 10;
zstd = 0.3;
smoothness = 0.25;
[Xsurf,Ysurf,Zsurf,sel] = rand3Dsurface(Nf,xdim,ydim,zstd,smoothness);

% random sampled scene points
X = Xsurf(sel);
Y = Ysurf(sel);
Z = Zsurf(sel);

% points projected into random views
Nviews = 2; % # views
dist = 10;   % [m]
spread = 5; % [m]
noise = 1;  % [pixel]
Cov_rph = diag([1 1 3]*DTOR).^2;
[Rt,uu_noisy,vv_noisy,uu_true,vv_true] = rand_views(Nviews,dist,spread,noise,X,Y,Z,K);

% represent pose in form:
% P1 = K[I | 0]   P2 = K[R | t]
R1 = Rt(:,1:3);
t1 = Rt(:,4);
R2 = Rt(:,5:7);
t2 = Rt(:,8);
R_true = R2*R1';
rph_true = rot2rph(R_true);
t_true = t2 - R_true*t1;
t_hat = t_true/norm(t_true);

% plot configuration
figure(1);
surf(Xsurf,Ysurf,Zsurf); % scene
colormap autumn;
material dull;
shading interp;
lighting gouraud;
light;
hold on;
plot3(X,Y,Z,'b+'); % sampled scene points
plot_coordinate_frame(R1',-R1'*t1,'C1');
plot_coordinate_frame(R2',-R2'*t2,'C2');
hold off;
axis equal;
rotate3d on;

% gather image points
[u1,v1] = deal(uu_noisy(:,1),vv_noisy(:,1));
[u2,v2] = deal(uu_noisy(:,2),vv_noisy(:,2));
[u1_true,v1_true] = deal(uu_true(:,1),vv_true(:,1));
[u2_true,v2_true] = deal(uu_true(:,2),vv_true(:,2));
figure(2); plot(u1,v1,'.'); axis ij; axis image; title('C1');
figure(3); plot(u2,v2,'.'); axis ij; axis image; title('C2');
clear uu_* vv_*;
end

fprintf('\n');
fprintf('     t/norm(t)\t\t       rph\n');
fprintf('--------------------\t-----------------------\n');
fprintf('%+.2f  %+.2f  %+.2f\t%+.2f  %+.2f  %+.2f\n', ...
	t_hat(1),t_hat(2),t_hat(3),rph_true(1)*RTOD,rph_true(2)*RTOD,rph_true(3)*RTOD);


U1 = homogenize(u1,v1);
U2 = homogenize(u2,v2);
U1_true = homogenize(u1_true,v1_true);
U2_true = homogenize(u2_true,v2_true);

X1 = inv(K)*U1;
X2 = inv(K)*U2;
X1_true = inv(K)*U1_true;
X2_true = inv(K)*U2_true;

[x1,y1] = dehomogenize(X1);
[x2,y2] = dehomogenize(X2);
[x1_true,y1_true] = dehomogenize(X1_true);
[x2_true,y2_true] = dehomogenize(X2_true);



Q = transpose(chol(Cov_rph));
rph_noisy = rph_true + Q*randn(3,1);
R_noisy = rotxyz(rph_noisy);

fprintf('-------------------------------------------------\n');
%fprintf('inital guess       \t%+.2f  %+.2f  %+.2f\n', ...
%	rph_noisy(1)*RTOD,rph_noisy(2)*RTOD,rph_noisy(3)*RTOD);


test = 2;
if test == 1
  [R_est,t_est,e_est] = relorient_horn(x1,y1,x2,y2,R_noisy);
  rph_est = rot2rph(R_est);
  fprintf('%+.2f  %+.2f  %+.2f\t%+.2f  %+.2f  %+.2f\n', ...
	  t_est(1),t_est(2),t_est(3),rph_est(1)*RTOD,rph_est(2)*RTOD,rph_est(3)*RTOD);
elseif test == 2
  [R,t,e_est] = relorient_sample(x1,y1,x2,y2,rph_true,Cov_rph,4);
  %rph_est = rot2rph(R_est);
  for i=1:length(e_est)
    e = e_est(i);
    R_est = R(:,[1 2 3]+3*(i-1));
    t_est = t(:,i);
    rph_est = rot2rph(R_est);
    fprintf('%+.2f  %+.2f  %+.2f\t%+.2f  %+.2f  %+.2f\t%.3e\n', ...
  	    t_est(1),t_est(2),t_est(3),rph_est(1)*RTOD,rph_est(2)*RTOD,rph_est(3)*RTOD,e);
  end

end
