% to test harris+zernike with matlab
% load from _test_corr_files and run matlab functions
%
% usage:
%  $ cd "to van direcotry"
%  $ addpath(genpath(pwd));
%  $ cd "to ${PERLS_DIR}/opt/examples/libvision/"
%  $ test_corr

function A_nm = test_zernike (t_nsamp, r_nsamp)

perls_dir = '~/perls/';

% i assume you have perls symbolic link in your home directory
data_dir = [perls_dir,'opt/examples/libvision/_test_corr_files/'];

uv= load([data_dir,'harris_uv.txt']);
u=uv(1,:)'; v=uv(2,:)';
n = length (u);
k = load([data_dir,'k.txt']);
K = reshape(k,3,3)';

order = 24; w=16;

x12 = load([data_dir,'x12.txt']);
R_lc = rotxyz (x12(4:6));

!rm -rf _test_zernike_files
!mkdir _test_zernike_files

% report the param
info = [t_nsamp, r_nsamp, order, w, n];
save _test_zernike_files/zernike_ans_info.txt info -ASCII -DOUBLE

% report the polar patch
Zbasis = zernikeBasisFcnsPolar([t_nsamp,r_nsamp],order); % [128,32]

xsamp = Zbasis.xsamp(:);
ysamp = Zbasis.ysamp(:);
sampler = [xsamp'; -ysamp';].*w;
save _test_zernike_files/sampler.txt sampler -ASCII -DOUBLE

[Zernike.u,Zernike.v] = deal(u, v);

img = imread([data_dir,'zernike_8bit_gray.png']);

[patchmat] = zernikepolar_featpatch_tester (img, Zernike.u, Zernike.v, Zbasis, ...
					                        K, R_lc, w);

save _test_zernike_files/patch.txt patchmat -ASCII -DOUBLE

% encode patches with normalized Zernike moments
[A_nm, V_nm] = zernikeMomentsPolar_tester(patchmat,Zbasis,'xcorr');

A_nm_real = real (A_nm); A_nm_imag = imag (A_nm);
V_nm_real = real (V_nm); V_nm_imag = -imag (V_nm);

save _test_zernike_files/vnm_real.txt V_nm_real -ASCII -DOUBLE
save _test_zernike_files/vnm_imag.txt V_nm_imag -ASCII -DOUBLE
save _test_zernike_files/anm_real.txt A_nm_real -ASCII -DOUBLE
save _test_zernike_files/anm_imag.txt A_nm_imag -ASCII -DOUBLE
