clear all;
% relative pose of camera 2
%t   = [1, 1, 1]';
%rph = [0, 0, 0]';
t = rand(3,1).*[1;1;0];
rph = rand(3,1).*[0.25; 0.25; 1]*pi;
R   = rotxyz(rph);

% construct a random scene
N = 20;
xdim = 1; ydim = 1; zstd = 0.5; smoothness = 0.7;
[X_mat,Y_mat,Z_mat,sel] = rand3Dsurface(N,xdim,ydim,zstd,smoothness);
Z_mat = Z_mat + 3;
Xs = [X_mat(sel)'; Y_mat(sel)'; Z_mat(sel)'];

% construct random points in the epipolar plane
Xe = repmat(rand(1,N)+1,[3 1]).*Xs  + 5*repmat(rand(1,N),[3 1]).*repmat(t,[1 N]);

% construct camera projection matrices
K = [1620   0      640; ...  % camera calibration matrix
     0      1620   512; ...
     0      0      1     ];

P1 = K*[eye(3), [0; 0; 0;]]; % camera 1 projection matrix
P2 = K*[R, t];               % camera 2 projection matrix

% project the two point clouds into each image plane
XXs = homogenize(Xs(1,:)',Xs(2,:)',Xs(3,:)');
XXe = homogenize(Xe(1,:)',Xe(2,:)',Xe(3,:)');
U1s = P1*XXs;  % scene projected image points
U2s = P2*XXs;

U1e = P1*XXe;  % consistent epipolar geometry
U2e = P2*XXe;  % projected points

% dehomogenize
[u1s,v1s] = dehomogenize(U1s);
[u2s,v2s] = dehomogenize(U2s);
[u1e,v1e] = dehomogenize(U1e);
[u2e,v2e] = dehomogenize(U2e);

% add noise
if true
  sigma = 1; % pixels
  u1s = u1s + sigma*rand(size(u1s));
  v1s = v1s + sigma*rand(size(v1s));
  u1e = u1e + sigma*rand(size(u1e));
  v1e = v1e + sigma*rand(size(v1e));  
end

% generate putative correspondeces
po = 0.8;         % percent outliers
osel = unique(ceil(N*rand(round(po*N),1))); % outlier index
isel = setdiff([1:N]',osel);                % inlier index
No = length(osel); % number of outliers
Ni = length(isel); % number of inliers
po = No/N;         % update percent outliers

u1 = u1s; v1 = v1s;
u2 = u2s; v2 = v2s;
u1(osel) = u1e(osel); % outliers consistent with
v1(osel) = v1e(osel); % epipolar geometry


% calculate normalize image coordinates
U1 = homogenize(u1,v1);
U2 = homogenize(u2,v2);
X1 = inv(K)*U1;
X2 = inv(K)*U2;
[x1,y1] = dehomogenize(X1);
[x2,y2] = dehomogenize(X2);

% find inlier set using essential matrix model fitting
% LMedS
[e_min,iselA,iterations] = estim_E_LMedS(x1,y1,x2,y2,0.6);
oselA = setdiff([1:N]',iselA);
% RANSAC
[e_min,iselB,iterations] = estim_E_RANSAC(x1,y1,x2,y2);
oselB = setdiff([1:N]',iselB);


% find inlier set using consistent motion constraint
[y_lsq,FOELAG_lsq] = motioncenter_lsq(u1,v1,u2,v2);
[y_est,rediuals,FOELAG_est,oselC] = motioncenter_mest(y_lsq,u1,v1,u2,v2);
iselC = setdiff([1:N]',oselB);

% show results
if true
  figure(1); clf;
  hold on;
  % plot putiative set
  plot_motion([],u1(isel),v1(isel),u2(isel),v2(isel),[],{'Color','g'});
  plot_motion([],u1(osel),v1(osel),u2(osel),v2(osel),[],{'Color','r'});
  hold off;
  title('Putative Set Consistent with Epipolar Geometry');
  
  figure(2); clf;
  hold on;
  % plot putiative set  
  plot_motion([],u1(isel),v1(isel),u2(isel),v2(isel),[],{'Color','g'});
  plot_motion([],u1(osel),v1(osel),u2(osel),v2(osel),[],{'Color','r'});

  % plot robust esimated inlier & outlier set
  %plot_motion([],u1(iselA),v1(iselA),u2(iselA),v2(iselA),[],{'Color',[1,1,1]*.75});
  plot_motion([],u1(oselA),v1(oselA),u2(oselA),v2(oselA),[],{'Color','y'});
  hold off;
  title('LMedS Essential Matrix Inlier Set');
  

  figure(3); clf;
  hold on;
  % plot putiative set  
  plot_motion([],u1(isel),v1(isel),u2(isel),v2(isel),[],{'Color','g'});
  plot_motion([],u1(osel),v1(osel),u2(osel),v2(osel),[],{'Color','r'});

  % plot robust esimated inlier & outlier set
  %plot_motion([],u1(iselB),v1(iselB),u2(iselB),v2(iselB),[],{'Color',[1,1,1]*.75});
  plot_motion([],u1(oselB),v1(oselB),u2(oselB),v2(oselB),[],{'Color','y'});
  hold off;
  title('RANSAC Essential Matrix Inlier Set');

  figure(4); clf;
  hold on;
  % plot putiative set  
  plot_motion([],u1(isel),v1(isel),u2(isel),v2(isel),[],{'Color','g'});
  plot_motion([],u1(osel),v1(osel),u2(osel),v2(osel),[],{'Color','r'});

  % plot robust esimated inlier & outlier set
  %plot_motion([],u1(iselC),v1(iselC),u2(iselC),v2(iselC),[],{'Color',[1,1,1]*.75});
  plot_motion([],u1(oselC),v1(oselC),u2(oselC),v2(oselC),[],{'Color','y'});
  hold off;
  title('Consistent Vector Motion Inlier Set');


end
