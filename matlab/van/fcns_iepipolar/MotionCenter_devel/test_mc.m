% SETUP SCENE AND MOTION
if true
  K = [1619.3 0      632.6; ...
       0      1617.6 511.9; ...
       0      0      1     ];

  %rph = [0,0,0];
  rph = 2*(rand(3,1)-0.5).*[5; 5; 180];
  R = rotxyz(rph*DTOR);
  %t = [0 0 1]';
  t = rand(3,1).*[1; 1; 0.25];
  
  P1 = K*[eye(3), [0 0 0]'];
  P2 = K*[R, t];

  [X_mat,Y_mat,Z_mat,sel] = rand3Dsurface(50,1,1,0.5,0.7);
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
    osel = unique(ceil(rand(p*N,1)*N));
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
[y_t_lsq,FOEFLAG_t_lsq] = motioncenter_lsq(u1_t,v1_t,u2_t,v2_t);
[y_m_lsq,FOEFLAG_m_lsq] = motioncenter_lsq(u1,v1,u2,v2);

% optimized result
%[y_t_est,residuals_t_est,FOEFLAG_t_est,osel_t_est] = motioncenter_mest(y_t_lsq,u1_t,v1_t,u2_t,v2_t);
[y_m_est,residuals_m_est,FOEFLAG_m_est,osel_m_est] = motioncenter_mest(y_m_lsq,u1,v1,u2,v2);

% inliers according to optimized results
isel_m_est = setdiff([1:N],osel_m_est);

osel   = reshape(osel,1,[]);                        % outlier index
ofound = reshape(intersect(osel_m_est,osel),1,[]);  % outliers found
ifound = reshape(intersect(isel_m_est,isel),1,[]);  % inliers found
omissed= reshape(setdiff(osel,osel_m_est),1,[])     % outliers missed
imissed= reshape(setdiff(isel,isel_m_est),1,[])     % inliers missed

Ni  = length(isel);    % number of inliers in point set
No  = length(osel);    % number of outliers in point set
Nif = length(ifound);  % number of inliers found
Nof = length(ofound);  % number of outliers found
Nom = length(omissed); % number of outliers missed
Nim = length(imissed); % number of inliers missed

% print results
fprintf('\n');
fprintf('ifound = %d of %d  ofound = %d of %d\n\n',Nif,Ni,Nof,No);

fprintf('rph = [%.1f %.1f %.1f]  t = [%.1f %.1f %.1f]\n',rph,t);
fprintf('-------------------------------------------------------\n');
if FOEFLAG_t_lsq == false
  fprintf('Center of Motion\t\t');
else
  fprintf('Focus of Expansion\t\t');
end
if FOEFLAG_m_lsq == false
  fprintf('Center of Motion\n');
else
  fprintf('Focus of Expansion\n');
end
fprintf('y_t_lsq = [%.2f %.2f %.2f] y_m_lsq = [%.2f %.2f %.2f]\n', ...
	y_t_lsq/y_t_lsq(3),y_m_lsq/y_m_lsq(3));
fprintf('-------------------------------------------------------\n');
if FOEFLAG_t_est == false
  fprintf('Center of Motion\t\t');
else
  fprintf('Focus of Expansion\t\t');
end
if FOEFLAG_m_est == false
  fprintf('Center of Motion\n');
else
  fprintf('Focus of Expansion\n');
end
fprintf('y_t_est = [%.2f %.2f %.2f] y_m_est = [%.2f %.2f %.2f]\n', ...
	y_t_est/y_t_est(3),y_m_est/y_m_est(3));
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

figure(3); clf;
hold on;
legend_str = {};
% plot y_t_lsq
plot(y_t_lsq(1)/y_t_lsq(3),y_t_lsq(2)/y_t_lsq(3),'b*');
legend_str{end+1} = 'y\_t\_lsq';
% plot y_m_lsq
plot(y_m_lsq(1)/y_m_lsq(3),y_m_lsq(2)/y_m_lsq(3),'r*');
legend_str{end+1} = 'y\_m\_lsq';
% plot y_t_est
plot(y_t_est(1)/y_t_est(3),y_t_est(2)/y_t_est(3),'b+');
legend_str{end+1} = 'y\_t\_est';
% plot y_m_est
plot(y_m_est(1)/y_m_est(3),y_m_est(2)/y_m_est(3),'r+');
legend_str{end+1} = 'y\_m\_est';

legend(legend_str{:});
% plot inliers
plot_motion([],u1(isel),v1(isel),u2(isel),v2(isel),[],{'Color','g'});
% plot outliers
plot_motion([],u1(osel),v1(osel),u2(osel),v2(osel),[],{'Color','r'});
hold off;
title('Motion of Putative Set');

figure(4); clf;
hold on;
% TRUTH
% plot inliers
plot_motion([],u1(isel),v1(isel),u2(isel),v2(isel),[],{'Color','g'});
% plot outliers
plot_motion([],u1(osel),v1(osel),u2(osel),v2(osel),[],{'Color','r'});
% ESTIMATED
% plot inliers
plot_motion([],u1(ifound),v1(ifound),u2(ifound),v2(ifound),[],...
	    {'Color',[1 1 1]*.75});
% plot outliers
plot_motion([],u1(ofound),v1(ofound),u2(ofound),v2(ofound),[],...
	    {'Color','y'});
hold off;
