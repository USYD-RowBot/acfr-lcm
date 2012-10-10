function twoviewCompare(fni,fnj,TheJournal,TheBathy,TheConfig);


if isempty(TheJournal.Ekf.mu);
  TheJournal.Ekf.mu = TheJournal.Eif.mu;
  TheJournal.Ekf.Sigma = TheJournal.Eif.Sigma;
  Xfi = TheJournal.Index.Xf_ii{fnj}(1:6);
  TheJournal.Ekf.Sigma(:,Xfi) = TheJournal.Eif.SigmaCol(:,1:6);
  TheJournal.Ekf.Sigma(Xfi,:) = TheJournal.Eif.SigmaCol(:,1:6)';
end;

% assign pointers
Calib = TheConfig.Calib;

%====================================================
% LOAD IMAGE DATA
%====================================================
% featureLUT is a feature look-up-table of image numbers organized feature number
Image1 = initializeImage(TheJournal.Index.featureLUT(fni),TheConfig);
Image2 = initializeImage(TheJournal.Index.featureLUT(fnj),TheConfig);


%====================================================
% LOAD/GENERATE IMAGE FEATURE DATA
%====================================================
% return a feature structure containing interest points and descriptors
Features1 = initializeFeatures(Image1,[],TheConfig);
Features2 = initializeFeatures(Image2,[],TheConfig);

% RDI measured bathymetry structure associated with each image
bathy1_t = TheBathy(fni);
bathy2_t = TheBathy(fnj);

% assign each image feature point a scene depth prior
%----------------------------------------------------
pixeltol = 100;
% zernike features
[Features1.Zernike.Z,Features1.Zernike.Cov_Z] = ...
    feature_depth_prior(Features1.Zernike.uc, Features1.Zernike.vc, ...
			bathy1_t.uc, bathy1_t.vc, bathy1_t.Z, bathy1_t.Cov_Z, pixeltol);
[Features2.Zernike.Z,Features2.Zernike.Cov_Z] = ...
    feature_depth_prior(Features2.Zernike.uc, Features2.Zernike.vc, ...
			bathy2_t.uc, bathy2_t.vc, bathy2_t.Z, bathy2_t.Cov_Z, pixeltol);

u1 = Features1.Zernike.uc; v1 = Features1.Zernike.vc; Z1 = Features1.Zernike.Z;  Cov_Z1 = Features1.Zernike.Cov_Z;
u2 = Features2.Zernike.uc; v2 = Features2.Zernike.vc; Z2 = Features2.Zernike.Z;  Cov_Z2 = Features2.Zernike.Cov_Z;
    
for ii=1:3;
  switch ii;
  case 1; TheConfig.Estimator.inferenceEif = on;   % seifs
          TheConfig.Estimator.useSeifsDA   = on;
  case 2; TheConfig.Estimator.inferenceEif = off;  % eif
  case 3; TheConfig.Estimator.inferenceEif = on;   % ci
          TheConfig.Estimator.useSeifsDA   = off;
  end;
   
  % extract pose information from our state estimate
  [x_lv1,x_lv2,x_vc,Sigma{ii}] = extract_poses(fni,fnj,TheConfig);

  
  % predicted relative poses and 1st order covariances assuming
  % Sigma elements are arranged in the order: [x_lv1,x_lv2,x_vc]
  %---------------------------------------------------
  % camera 2 w.r.t camera 1
  [x_c1c2,J] = relative_sensor_pose(x_lv1,x_lv2,x_vc);
  P_c1c2{ii} = spdproduct(Sigma{ii}(1:12,1:12),J(:,1:12)');
  % camera 1 w.r.t camera 2
  [x_c2c1,J] = relative_sensor_pose(x_lv2,x_lv1,x_vc);
  J = J(:,[7:12,1:6,13:18]); % re-arrange jacobian to match order of Sigma
  P_c2c1{ii} = spdproduct(Sigma{ii}(1:12,1:12),J(:,1:12)');

  %if ii==3; keyboard;end;
  
  %======================================================
  % RESTRICT FEATURE CORRESPONDENCE BASED UPON PRIOR POSE
  %======================================================  
  % transfer feature points from image 1 into image 2 based upon
  % scene depth and pose prior information
  %------------------------------------------------------  
  % predicted location of feature points (u1,v1) in image 2
  % NOTE that i'm assigning *ZERO* uncertainty for the camera calibration
  % matrix, i am treating it as a known quantity
  [u2p,v2p,Cov_u2pv2p{ii}] = relview_ptxfer2(Calib.K, x_c2c1, Z1, u1, v1, P_c2c1{ii}, 20);
  %[u2p,v2p,Cov_u2pv2p{ii}] = relview_ptxfer2(Calib.K, x_c2c1, Z1, u1, v1, P_c2c1{ii}, Cov_Z1(1));
  % predicted location of feature points (u2,v2) in image 1
  %[u1p,v1p,Cov_u1pv1p{ii}] = relview_ptxfer2(Calib.K, x_c1c2, Z2, u2, v2, P_c1c2{ii}, Cov_Z2(1));
  % specify confidence level alpha
  alpha = 1-2*normcdf(-8);
  fudge = 1;
  chiSquare2dof = chi2inv(alpha,2) * fudge^2;  
end;

% fundamental matrix based upon prior pose
%------------------------------------------------------
% prior-pose fundamental matrix satisfying u2'*F*u1 = 0
t = x_c2c1(1:3);
R = rotxyz(x_c2c1(4:6));
Fpp21 = F_from_KRt(R,t,Calib.K);

% plot pose bounded search region when transfering points (u1,v1) to (u2p,v2p)
figure(100);
sample_ellipses(Image1.Iwarp,Image2.Iwarp,u1,v1,u2,v2,u2p,v2p,Cov_u2pv2p,Fpp21,chiSquare2dof);
figure(100);
title('Pose Prior Ellipse I1 with sampling of (u1,v1)');
set(100,'Name','Pose Prior Corr Ellipse I1');
figure(101); 
title('Pose Prior Ellipse I2 with sampling of (u2p,v2p)');
set(101,'Name','Pose Prior Corr Ellipse I2');



function sample_ellipses(I1,I2,u1,v1,u2,v2,u2p,v2p,Cov_u2pv2p,F21,chiSquare2dof)

fignum = gcf;
nr = size(I1,1);
nc = size(I1,2);

udata = [1 nc]-1;
vdata = [1 nr]-1;

% sample I1 spatially
%-------------------------------------------
nx = 1;
ny = 3;
nsamps = nx*ny;
xsamp1 = round(linspace(0.1*nc,0.9*nc,nx));
ysamp1 = round(linspace(0.1*nr,0.9*nr,ny));
[xsamp1,ysamp1] = meshgrid(xsamp1, ysamp1);
xsamp1 = xsamp1(:);
ysamp1 = ysamp1(:);

% find closest interest points to samples
%-------------------------------------------
tri = delaunay(u1,v1);
sel = dsearch(u1,v1,tri,xsamp1,ysamp1);
u1 = u1(sel);
v1 = v1(sel);
u2p = u2p(sel);
v2p = v2p(sel);
Cov_u2pv2p{1} = Cov_u2pv2p{1}(:,:,sel);
Cov_u2pv2p{2} = Cov_u2pv2p{2}(:,:,sel);
Cov_u2pv2p{3} = Cov_u2pv2p{3}(:,:,sel);


% plot pose instantiated epipolar lines
%--------------------------------------------
figure(fignum); clf;
set(fignum,'DoubleBuffer','on');
imagesc(udata,vdata,I1); colormap gray; axis image off; title('I1');
hold on;
cvec = draw_epipolar_lines(F21',u2p,v2p,fignum,[nr nc],'g');
hold off;

figure(fignum+1);
set(fignum+1,'DoubleBuffer','on');
imagesc(udata,vdata,I2); colormap gray; axis image off; title('I2');
hold on;
cvec = draw_epipolar_lines(F21,u1,v1,fignum+1,[nr nc],'g');
hold off;

% plot interest points (u1,v1) on I1
%----------------------------------------
% note in this case it's faster to use two "for loops" vs. a single "for
% loop".  this is because it slows matlab down to switch back and forth
% between the two figure windows.
figure(fignum);
hold on;
for ii = 1:nsamps
  plot(u1(ii),v1(ii),'+','color',cvec(ii,:),'linewidth',1.75);
  plot(u1(ii),v1(ii),'o','color',cvec(ii,:),'linewidth',1.75);
end
hold off;

% plot uncertainty ellipses based upon results of two-vew point transfer
%-----------------------------------------------------------------------
figure(fignum+1);
hold on;
% bounding ellipse is specified by chi-squared random variable
% with 2 degrees of freedom
% chiSquare2dof is chi squared random variable with 2 DOF
for ii = 1:nsamps
  % plot the predicted point (u2p,v2p) on I2
  plot(u2p(ii),v2p(ii),'+','color',cvec(ii,:),'linewidth',1.75);
  plot(u2p(ii),v2p(ii),'o','color',cvec(ii,:),'linewidth',1.75);
  for kk=1:3
    % plot the search ellipse centered on (u2p,v2p) on I2
    Sigma = squeeze(Cov_u2pv2p{kk}(:,:,ii));
    switch kk;
    case 1; color = 'r'; % seifs
    case 2; color = 'y'; % eif
    case 3; color = 'b'; % ci
    end;
    draw_ellipse([u2p(ii); v2p(ii)],Sigma,chiSquare2dof,'--','color',color);
  end;
end;
hold off;

set(fignum,'DoubleBuffer','off');
set(fignum+1,'DoubleBuffer','off');
