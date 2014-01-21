% to test pccs with matlab
% load from _test_corr_files and run matlab functions
%
% usage:
%  $ cd "to van direcotry"
%  $ addpath(genpath(pwd));
%  $ cd "to perls/opt/examples/libvision/"
%  $ test_corr

function test_corr

dir = '~/perls/src/examples/libvision/_test_corr_files/';
k = load([dir,'k.txt']);
key1 = load([dir,'key1.txt']);
key2 = load([dir,'key2.txt']);
x12 = load([dir,'x12.txt']);
x21 = load([dir,'x21.txt']);
p12 = load([dir,'p12.txt']);
p21 = load([dir,'p21.txt']);
uv1 = load([dir,'uv1.txt']);
uv2 = load([dir,'uv2.txt']);
z1 = load([dir,'z1.txt']);
z2 = load([dir,'z2.txt']);

n =1000;
uv1 = reshape(uv1,n,2)';
uv2 = reshape(uv2,n,2)';
key1 = reshape(key1,n,128)';
key2 = reshape(key2,n,128)';
K = reshape(k,3,3)';
p12 = reshape(p12,6,6)';
p21 = reshape(p21,6,6)';
Z1 = z1; Z2 = z2;
u1 = uv1(1,:)'; v1 = uv1(2,:)';
u2 = uv2(1,:)'; v2 = uv2(2,:)';

%% test simscore pccs
Image1=[]; Image2=[]; TheConfig.Calib.K=K;
minmaxString = 'min';
x_v1c = zeros(6,1);
x_v2c = zeros(6,1);
Cov_Z1=0.01*eye(2); Cov_Z2=0.01*eye(2);

keys1.descriptor = key1;
keys2.descriptor = key2;
keys1.num = n;
keys2.num = n;

smatrix = siftSimilarityScore(keys1,keys2);
TheConfig.Plot.twoview_pt_xfer = off;

simAB = 0.85;

Image1.Iwarp =  [];
Image2.Iwarp =  [];
Image1.imgnum = 0;
Image2.imgnum = 0;

tic
[sel1,sel2,simA,simB] = ...
    putative_corrset(u1, v1, u2, v2,  Z1, Z1, Cov_Z1, Cov_Z2, smatrix, minmaxString, ...
		     x12, x21, p12, p21, Image1, Image2, TheConfig);
         
tmp = find(simA < simAB^2*simB); % 0.6 is recommended by Lowe 0.7 for lowe sift van
pselSift1 = sel1(tmp); pselSift2 = sel2(tmp);
toc

n_corr = length(pselSift1);

info = [n_corr, simAB];

save _test_corr_files/ans_info.txt info -ASCII -DOUBLE
save _test_corr_files/ans_pccs_sel1.txt pselSift1 -ASCII -DOUBLE
save _test_corr_files/ans_pccs_sel2.txt pselSift2 -ASCII -DOUBLE

%% test estim E and H
D_E = load([dir,'D.txt']);
[estruc_samp,n_e] = estim_E_6p(D_E);

  
%% 
u1 = u1(pselSift1);
v1 = v1(pselSift1);
u2 = u2(pselSift2);
v2 = v2(pselSift2);

invK = inv(K);
X1 = invK*homogenize(u1,v1);
X2 = invK*homogenize(u2,v2);
[x1,y1] = dehomogenize(X1);
[x2,y2] = dehomogenize(X2);

[hMatrix_test,sel_test_H,iter_test,gic_h] = estim_H_RANSAC(x1,y1,x2,y2);    
[eMatrixVec_test,sel_test_E,iter_test,gic_e] = estim_E_RANSAC(x1,y1,x2,y2);

x_test1 = u1(sel1); y_test1 = v1(sel1); x_test2 = u2(sel2); y_test2 = v2(sel2);
H=K*hMatrix_test*invK;
F = invK'*reshape(eMatrixVec_test,3,3)'*invK;
geo_H = sum(geo_distance_H(H,x_test1(sel_test_H),y_test1(sel_test_H),x_test2(sel_test_H),y_test2(sel_test_H)));
geo_E = sum(geo_distance_E(F,x_test1(sel_test_E),y_test1(sel_test_E),x_test2(sel_test_E),y_test2(sel_test_E)));

n_H = length(sel_test_H);
n_E = length(sel_test_E);

gic_H = geo_H + 2*2*n_H + 8*4;
gic_E = geo_E + 2*3*n_E + 5*4;