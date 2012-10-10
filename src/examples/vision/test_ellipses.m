% to test pccs search bound with matlab
% load from _test_ellipses_files and run matlab functions

% Usage: 
% Modify definition in pccs.c to write files into debug folder.
% run this script to see error in projected covariance.
% add van trunk directory by running before running the script
% >> addpath(genpath('../../../matlab/van/'))

function test_ellipses

dir = '~/perls/opt/examples/libvision/_test_ellipses_files/';
% dir = '~/perls/bin/_test_ellipses/';

% TheConfig.Data.imageCalibrationFile = [dir,'calib_huls3.m'];
% TheConfig.Calib = initializeCalib(TheConfig.Data.imageCalibrationFile);
% K = TheConfig.Calib.K;

K = load([dir,'K.txt']);
x21 = load([dir,'x21.txt']);
p21 = load([dir,'p21.txt']);
uv1 = load([dir,'uv1.txt']);
uv2 = load([dir,'uv2.txt']);
z1 = load([dir,'z1.txt']);
cov2p = load([dir,'cov2p.txt']);

n =length(uv1)/2;
K = reshape(K,3,3)';
uv1 = reshape(uv1,n,2)';
uv2 = reshape(uv2,n,2)';
p21 = reshape(p21,6,6)';
u1 = uv1(1,:)'; v1 = uv1(2,:)';
u2 = uv2(1,:)'; v2 = uv2(2,:)';
cov2p = reshape (cov2p, 8,4)';

alpha = 1-2*normcdf(-6);
fudge = 1;
chiSquare2dof = chi2inv(alpha,2) * fudge^2;

Cov_Z1=0.01*eye(2);

% images
I = imread('tire.tif'); J=imresize(I, [1024,1360]);
Image1.Iwarp =  J; Image2.Iwarp =  J;
Image1.imgnum = 0; Image2.imgnum = 0;

%% run
tic
[u2p,v2p,Cov_u2pv2p] = relview_ptxfer2(K, x21, z1, u1, v1, p21, Cov_Z1(1));
toc

%% verify
error = 0;
for ii=1:8
    error = error + sum(abs(cov2p(:,ii)-reshape(Cov_u2pv2p(:,:,ii),4,1)));
end

fprintf (1, 'total sum of error = %g\n', error);
%% plot
t = x21(1:3);
R = rotxyz(x21(4:6));
Fpp21 = F_from_KRt(R,t,K);
figure(30);
sample_ellipses(Image1.Iwarp,Image2.Iwarp,u1,v1,u2,v2,u2p,v2p,Cov_u2pv2p,Fpp21,chiSquare2dof);
figure(30);
title('Pose Prior Ellipse I1 with sampling of (u1,v1)');
set(30,'Name','Pose Prior Corr Ellipse I1');
