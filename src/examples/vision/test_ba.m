% to test sba with matlab
% generate synthetic points and project to each image
% usage:
%  $ cd "to van direcotry"
%  $ addpath(genpath(pwd));
%  $ test_ba(std, 1) to generate data for 3D with noise std
%  $ test_ba(std, 0) to generate data for 3D with noise std

function [x_5dof, p, S] = test_ba(std,e,x21)
% addpath(genpath('~/perls/matlab/van'))
if nargin == 0
   std = 1;
   e = 1;
end
if (nargin < 3)
    x21 = [0.3,0.1, 0.1, [1, -10, 0]*DTOR]'; % xyzrph
end

%include a percentage of outliers
outlier_pct = 0.01;

if e == 1
    dof = 3; % 3D sturcture
else
    dof = 2; % plane
end



b = trans2dm(x21(1:3));
x_5dof = [b(1:2); x21(4:6);];

h=0.1; offset = 2;
[x3d idx] = test_ba_buildpts(dof,h,offset);
x3d_h = [x3d; ones(1,size(x3d,2))];

%% (2) project to uv1 and uv2
% calib = initializeCalib('hauv/calib_hauv');
K=[1717.61 0 716.482; 0 1722.09 553.074; 0 0 1;];
t = x21(1:3); rph=x21(4:6); R=rotxyz(rph);
P1=K*[eye(3) zeros(3,1)]; P2=K*[R t];

imgwidth = 1380; imgheight = 1024;

% std=0;
x1 = P1*x3d_h; x2 = P2*x3d_h;
[u1,v1]=dehomogenize(x1);
[u2,v2]=dehomogenize(x2);
rmidx = union(find (0 > u1 | u1 > imgwidth),find (0 > v1 | v1 > imgheight));
u1(rmidx)=[]; v1(rmidx)=[];
x3d(:,rmidx)=[];
idx1=idx; idx1(rmidx)=[];
rmidx = union(find (0 > u2 | u2 > imgwidth),find (0 > v2 | v2 > imgheight));
u2(rmidx)=[]; v2(rmidx)=[];
idx2=idx; idx2(rmidx)=[];

% add in noise
u1=u1+std.*randn(size(u1,1),1);
v1=v1+std.*randn(size(v1,1),1);
u2=u2+std.*randn(size(u2,1),1);
v2=v2+std.*randn(size(v2,1),1);

% add in outliers (randomly swap two elements in the first u1 v1, this
% produces two outliers
for k = 1:floor(outlier_pct*size(u1,1)/2)
    swap = randsample(size(u1,1),2);
    tmp_u = u1(swap(1)); tmp_v = v1(swap(1));
    u1(swap(1)) = u1(swap(2));
    v1(swap(1)) = v1(swap(2));
    u1(swap(2)) = tmp_u;
    v1(swap(2)) = tmp_v;
end


%% (3) select pair
[common_idx, psel1, psel2]=intersect(idx1,idx2);
nsample = 200; %3000
%ii = ceil(size(common_idx,2).*rand(nsample,1)); %samples with replacement
ii = randsample(size(common_idx,2),min(nsample,size(common_idx,2)));
% load ii_synthpeak;
psel1=psel1(ii); psel2=psel2(ii);

%% (4) plot
% uncomment this section if you want to see figures
mksize_gnd = 2; %20;
mksize_smp = 10; %30;

figure(1); plot3(x3d(1,:),x3d(2,:),x3d(3,:),'.','MarkerSize',mksize_gnd);
scale = 0.3; color='ggmk';
plot_coordinate_frame(eye(3) ,zeros(3,1),'C1',scale,color);
plot_coordinate_frame(R,t,'C2',scale,color);
grid on;

figure(2); plot(u1,v1,'.','MarkerSize',mksize_smp);
hold on; plot(u1(psel1),v1(psel1),'r.','MarkerSize',mksize_smp); 
axis([0,imgwidth,0,imgheight]); hold off;
figure(3); plot(u2,v2,'.','MarkerSize',mksize_smp);
hold on; plot(u2(psel2),v2(psel2),'r.','MarkerSize',mksize_smp); 
axis([0,imgwidth,0,imgheight]); hold off;

%% print & load
uv1 = [u1(psel1),v1(psel1)];
uv2 = [u2(psel2),v2(psel2)];
X1 = x3d(:,psel1)';
X1 = X1+std*ones(size(X1));
u1 = uv1(:,1); v1 = uv1(:,2); u2 = uv2(:,1); v2 = uv2(:,2);
t_o = x21(1:3); rph_o = x21(4:6);

uv1_export = uv1';
uv2_export = uv2';
X1_export = X1';

ninliers = length(psel1);
K_export = [reshape(K',9,1); e; ninliers;];

!rm -rf _test_ba_files
!mkdir _test_ba_files
save _test_ba_files/ba_k.txt K_export -ASCII -DOUBLE
save _test_ba_files/ba_uv1.txt uv1_export -ASCII -DOUBLE
save _test_ba_files/ba_uv2.txt uv2_export -ASCII -DOUBLE
save _test_ba_files/ba_X1.txt X1_export -ASCII -DOUBLE

if e == 1
    %% run RT
%     tic
    [p_21_rt,Cov_21_rt,X_mle,exitflag,output] = ...
        twoview_BundleAdjust_Rt(t_o,rph_o,X1',u1,v1,u2,v2,K);
%     toc
    [b,J] = TRANS2DM(p_21_rt(1:3));
    p_21_rt = [b(1:2); p_21_rt(4:6)];
    Cov_21_rt = [J zeros(3,3); zeros(3,3) eye(3)]*Cov_21_rt*[J zeros(3,3); zeros(3,3) eye(3)]';
    Cov_21_rt(3,:) = []; Cov_21_rt(:,3) = [];
    Cov_21_rt;

    %% run RAE
    tic
    [p_21_rae,Cov_21_rae,X_mle,exitflag,output] = ...
        twoview_BundleAdjust_Rae(t_o,rph_o,X1',u1,v1,u2,v2,K);
    toc
    p_21_rae
    Cov_21_rae
    
    %% run robust RAE
    tic
    [p_21_rae_robust,Cov_21_rae_robust,X_mle,exitflag,output] = ...
        twoview_BundleAdjust_Rae(t_o,rph_o,X1',u1,v1,u2,v2,K,@huber_weight);
    toc
    p_21_rae_robust
    Cov_21_rae_robust
    
    try 
        chol(Cov_21_rae);
    catch
        display ('not pos def\n');
    end

    scale = norm(t_o);
    b_21_rae = [p_21_rae(1:2); scale];
    t_21_rae = dm2trans(b_21_rae);
    p_21_6dof = [t_21_rae; p_21_rae(3:5)];
    save _test_ba_files/ba_p21.txt p_21_6dof -ASCII -DOUBLE    
    
    save _test_ba_files/ans_x21.txt p_21_rae -ASCII -DOUBLE    
    save _test_ba_files/ans_S21.txt Cov_21_rae -ASCII -DOUBLE  
    save _test_ba_files/ans_x21_robust.txt p_21_rae_robust -ASCII -DOUBLE    
    save _test_ba_files/ans_S21_robust.txt Cov_21_rae_robust -ASCII -DOUBLE  
    
    p = p_21_rae;
    S = Cov_21_rae;
else
    d = mean(X1(:,3));
%     d=6.03;
    n = [0 0 -1]';
    H=K*(R-t*n'./d)*inv(K);
    [u1_o,v1_o] = dehomogenize(inv(H)*homogenize(u2,v2));
    uv1p_export = [u1_o,v1_o]';
    save _test_ba_files/ba_uv1p.txt uv1p_export -ASCII -DOUBLE
 
%     tic
    [p_21_h,Cov_21_h,exitflag,output] = ...
        test_ba_H (t_o,rph_o,n,d,u1_o,v1_o,u1,v1,u2,v2,K);
%     toc
%     p_21_h
%     Cov_21_h

    save _test_ba_files/ans_x21.txt p_21_h -ASCII -DOUBLE    
    save _test_ba_files/ans_S21.txt Cov_21_h -ASCII -DOUBLE    
    
    p = p_21_h;
    S = Cov_21_h;
end

% change initial guess a bit
% x21 = [0.1, 0.2, 0, [1, -10, 0]*DTOR]'; % xyzrp
save _test_ba_files/ba_x21.txt x21 -ASCII -DOUBLE
save _test_ba_files/x_5dof.txt x_5dof -ASCII -DOUBLE

%compare errors between with and without robust cost funciton
pose_error =  sqrt(sum((x_5dof-p_21_rae).^2))
pose_error_robust = sqrt(sum((x_5dof-p_21_rae_robust).^2))

end

function c = huber_weight(c)
    k = 1.345; %recomonded tuning parameter from Zhang paper
    tmp_inds = find(abs(c) >= k);
    c(tmp_inds) = c(tmp_inds).*sqrt(k./abs(c(tmp_inds)));

    %mean(abs(c))
   
end

%currently causing an error need to look into why
% function c = tukey_weight(c)
%     k = 4.6851; %recomonded tuning parameter from Zhang paper
%     tmp_inds_lt = find(abs(c) <= k);
%     tmp_inds_gt = find(abs(c) > k);
%     c(tmp_inds_gt) = 0;
%     c(tmp_inds_lt) = c(tmp_inds_lt).*(1-(c(tmp_inds_lt)./k).^2);
% end
