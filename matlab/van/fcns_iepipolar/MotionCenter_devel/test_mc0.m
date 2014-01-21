% SETUP SCENE AND MOTION
if false
  K = [1619.3 0      632.6; ...
       0      1617.6 511.9; ...
       0      0      1     ];

  rph = [0,0,0];
  %rph = rand(3,1).*[5;5;360];
  R = rotxyz(rph*DTOR);
  t = [0 0 1]';
  %t = rand(3,1);
  
  P1 = K*[eye(3), [0 0 0]'];
  P2 = K*[R, t];

  [X_mat,Y_mat,Z_mat,sel] = rand3Dsurface(50,1,1,0.2,0.7);
  Z_mat = Z_mat + 3;
  X = X_mat(sel);
  Y = Y_mat(sel);
  Z = Z_mat(sel);

  XX = homogenize(X,Y,Z);
  
  U1 = P1*XX;
  U2 = P2*XX;

  [u1,v1] = dehomogenize(U1);
  [u2,v2] = dehomogenize(U2);

  nr = 1024;
  nc = 1280;
  sel = find((0 < u1) & (u1 < nc) & ...
	     (0 < v1) & (v1 < nr) & ...
	     (0 < u2) & (u2 < nc) & ...
	     (0 < v2) & (v2 < nr));

  X = X(sel); Y = Y(sel); Z = Z(sel);
  u1_t = u1(sel); v1_t = v1(sel);
  u2_t = u2(sel); v2_t = v2(sel);
  N = length(u1_t);
  
  
  % ADD NOISE AND OUTLIERS
  if true
    u1 = u1_t; v1 = v1_t;    
    u2 = u2_t; v2 = v2_t;

    % outliers
    p = 0.2; % percent outliers
    No = ceil(p*N);
    osel = unique(ceil(rand(No,1)*N));
    isel = setdiff([1:N]',osel);
    u1(osel) = u1_t(osel(end:-1:1));
    v1(osel) = v1_t(osel(end:-1:1));
    
    % noise
    sigma = 1;
    u1 = u1 + sigma*rand(size(u1_t));
    v1 = v1 + sigma*rand(size(v1_t));
    u2 = u2 + sigma*rand(size(u2_t));
    v2 = v2 + sigma*rand(size(v2_t));
  end
end


% least-squares motion center on truth and corrupted points
[uc_t_lsq,vc_t_lsq,residual_t_lsq] = motion_center_lsq(u1_t,v1_t,u2_t,v2_t);
[uc_lsq,vc_lsq,residual_lsq] = motion_center_lsq(u1,v1,u2,v2);

% least-squares motion center on truth and corrupted points
% using homogenous coordinates
y_t_lsq = motion_center_homo_lsq(u1_t,v1_t,u2_t,v2_t);
y_lsq = motion_center_homo_lsq(u1,v1,u2,v2);

% nonlinear least-squares motion center on truth and corrupted points
[uc_t_est,vc_t_est,residual_t_est] = motion_center([uc_t_lsq;vc_t_lsq],u1_t,v1_t,u2_t,v2_t);
[uc_est,vc_est,residual_est] = motion_center([uc_lsq;vc_lsq],u1,v1,u2,v2);

% nonlinear motion center on truth and corrupted points
% using homogenous coordinates
y_t_est = motion_center_homo(y_t_lsq,u1_t,v1_t,u2_t,v2_t);
y_est = motion_center_homo(y_lsq,u1,v1,u2,v2);

% nonlinear least-squares motion center on truth and corrupted points
[y2_t_est,residual_t_est] = motion_center3(y_t_lsq,u1_t,v1_t,u2_t,v2_t);
[y2_est,residual_est] = motion_center2([uc_lsq;vc_lsq],u1,v1,u2,v2);
%[y2_est,residual_est] = motion_center3(y_lsq,u1,v1,u2,v2);


% prune outliers based upon residual from tangnent
%[sel_true,theta_true] = prune_motion(u1_true,v1_true,u2_true,v2_true,uc_true,vc_true);
%[sel,theta] = prune_motion(u1,v1,u2,v2,uc,vc);


% print results
fprintf('\n');

stats = load('mc2.mat');

osel   = reshape(osel,1,[])
ofound = reshape(intersect(stats.ii,osel),1,[]);
ifound = reshape(intersect(stats.jj,isel),1,[]);
omissed= reshape(setdiff(osel,stats.ii),1,[])
imissed= reshape(setdiff(isel,stats.jj),1,[])

Ni  = length(isel);
No  = length(osel);
Nif = length(ifound);
Nof = length(ofound);
Nom = length(omissed);
Nim = length(imissed);

fprintf('ifound = %d of %d  ofound = %d of %d\n\n',Nif,Ni,Nof,No);

fprintf('rph = [%.1f %.1f %.1f]  t = [%.1f %.1f %.1f]\n', ...
	rph(1),rph(2),rph(3),t(1),t(2),t(3));
fprintf('-------------------------------------------------------\n');
fprintf('y_t_lsq = [%.2f %.2f]  y_lsq = [%.2f %.2f]\n', ...
	uc_t_lsq,vc_t_lsq,uc_lsq,vc_lsq);
fprintf('y_t_est = [%.2f %.2f]  y_est = [%.2f %.2f]\n', ...
	uc_t_est,vc_t_est,uc_est,vc_est);
fprintf('-------------------------------------------------------\n');
fprintf('yh_t_lsq = [%.2f %.2f %.2f] yh_lsq = [%.2f %.2f %.2f]\n', ...
	y_t_lsq(1),y_t_lsq(2),y_t_lsq(3),y_lsq(1),y_lsq(2),y_lsq(3));
fprintf('yh_t_est = [%.2f %.2f %.2f] yh_est = [%.2f %.2f %.2f]\n', ...
	y_t_est(1),y_t_est(2),y_t_est(3),y_est(1),y_est(2),y_est(3));
fprintf('-------------------------------------------------------\n');
fprintf('y2_t_est = [%.2f %.2f %.2f] y2_est = [%.2f %.2f %.2f]\n', ...
	y2_t_est(1),y2_t_est(2),y2_t_est(3),y2_est(1),y2_est(2),y2_est(3));
fprintf('\n');

figure(1); clf;
surf(X_mat,Y_mat,Z_mat); shading interp;
hold on;
plot3_multicolor(X,Y,Z);
hold off;
title('Scene and Feature Points');

figure(2); clf;
subplot(1,2,1);
plot_multicolor(u1,v1);
subplot(1,2,2);
plot_multicolor(u2,v2);
title('Putative Set');

figure(4); clf;
plot(uc_t_lsq,vc_t_lsq,'b*',uc_lsq,vc_lsq,'r*', ...
     uc_t_est,vc_t_est,'g*',uc_est,vc_est,'m*', ...
     y2_est(1)/y2_est(3),y2_est(2)/y2_est(3),'c*');
legend('y\_t\_lsq','y\_lsq','y\_t\_est','y\_est','y2\_est');
hold on;
plot_motion([],u1,v1,u2,v2);
hold off;
title('Motion of Putative Set');

figure(5); clf;
plot(1:N,residual_t_lsq,'b.',1:N,residual_lsq,'r.', ...
     1:N,residual_t_est,'g.',1:N,residual_est,'m.');
legend('residual\_t\_lsq','residual\_lsq','residual\_t\_est','residual\_est');

%figure(6); clf;
%plot(1:N,theta_true*RTOD,'r.',1:N,theta*RTOD,'b.');
%title('theta motion vector & tangent');
%legend('theta true','theta meas');
