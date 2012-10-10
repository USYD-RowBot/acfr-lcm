% tests the generation of moments and reconstruction of patch from
% the moments

phalf = 16;
zrad = 32;   % remember to synchronize with apatch_extract
tsamps = 128;% 8*zrad;  % zrad should be the same as radsamps
nmax = 20; %12
% load or generate the zernike polynomials
datfile = '/tmp/descrip_zernike_radial.mat';
if exist(datfile,'file')
  cmd = ['load ' datfile ';'];
  eval(cmd);
else
  display('Zernike polynomials file not found. Generating ...')
end
  
if (~exist('Zstruc') | Zstruc.nmax ~=nmax | Zstruc.zrad ~=zrad | ...
    Zstruc.tsamps ~= tsamps)

  [Zstruc.Z,Zstruc.ndesc] = zergen_radial(nmax,zrad,tsamps);
  Zstruc.zrad = zrad;
  Zstruc.nmax = nmax;  
  Zstruc.tsamps = tsamps;
end

if false;
  fpatch = double(imfilter(single(randn(2*phalf+1,2*phalf+1)),fspecial('gaussian',14,1)));

  i = 100;
  j = 100;
  % fpatch = I1(i-phalf:i+phalf,j-phalf:j+phalf);
  sigsmooth = 2*pi*phalf/tsamps /2; %/2
  %sigsmooth = 1;
  fsmooth = fspecial('gaussian',ceil(sigsmooth*7),sigsmooth);

  fpatch = double(imfilter(single(fpatch),fsmooth));
  
  fpatch = fpatch-mean(fpatch(:));
  fpatch = fpatch./(fpatch(:)'*fpatch(:))^0.5;
else;
  I = imread('cameraman.tif');
  %Ihead = I(32:96,96:160);
  Ihead = I([32:64]+15,[96:128]);  
  fpatch = double(Ihead) - mean(Ihead(:));
  fpatch = fpatch / sqrt(fpatch(:)'*fpatch(:));
end;


[fpolmat,U,V] = cart2polar(fpatch,zrad,tsamps);


fpolvec = fpolmat(:)'; % as a row vector
% moments in a structure
J = z_jet_radial(fpolvec,1,1,Zstruc);
% weighted moments (for correlation) for each feature as a row in V
[V,alpha,Vpos] = zpinvariants(J);

Avec = Vpos(1,:); % only the positive m;
fpolmat_recon = zreconstruct(Avec,nmax,tsamps,zrad,'corr');

samps = 101;
[row,col] = size(fpatch);
finterp = interp2(fpatch,1);

Zbasis = zernikeBasisFcns(size(finterp),nmax);

finterp = finterp - mean(finterp(Zbasis.ind));
finterp = finterp / sqrt(finterp(Zbasis.ind)'*finterp(Zbasis.ind));


A_nm   = zernikeMoments(finterp,Zbasis);
ftilde = zernikeReconstruct(A_nm,Zbasis);

% weight Zernike moments for correlation
%n = Zbasis.zindex(1,:)';
%m = Zbasis.zindex(2,:)';
%ii = find(m ~= 0);
%W_nm = A_nm .* sqrt(pi ./ (n+1)) / sqrt(Zbasis.darea);
%W_nm = [W_nm; conj(W_nm(ii))]; % add negative repetion moments

X(1) = finterp(Zbasis.ind)'*finterp(Zbasis.ind);
X(2) = ftilde(:)'*ftilde(:);
%X(3) = W_nm'*W_nm;
X(3) = zernikeCorrelationScore(A_nm,A_nm,Zbasis);
fprintf('\n');
fprintf('finterp(ind)''*finterp(ind) =%.4f\n',X(1));
fprintf('ftilde(ind)''*ftilde(ind)   =%.4f (%+.4f)\n',X(2),X(2)-X(1));
fprintf('W_nm''*W_nm                 =%.4f\n',X(3));
fprintf('\n');

Y(1) = fpolmat(:)'*fpolmat(:);
Y(2) = fpolmat_recon(:)'*fpolmat_recon(:);
Y(3) = V*V';
fprintf('fpolmat(:)''*fpolmat(:)            =%.4f\n',Y(1));
fprintf('fpolmat_recon(:)''*fpolmat_recon(:)=%.4f (%+.4f)\n',Y(2),Y(2)-Y(1));
fprintf('V''*V                              =%.4f\n',Y(3));

figure(1); imagesc(fpatch); colormap gray; colorbar;  climCart = get(gca,'clim');
figure(2); imagesc(fpolmat); colormap gray; colorbar; climPol  = get(gca,'clim');
figure(3); imagesc(ftilde,climCart); colormap gray; colorbar;
figure(4); imagesc(fpolmat_recon,climPol); colormap gray; colorbar;
fdiff  = zeros(Zbasis.patchSize);
fdiff(Zbasis.ind) = ftilde(Zbasis.ind)-finterp(Zbasis.ind);
figure(5); imagesc(fdiff,climCart); colormap gray; colorbar;
figure(6); imagesc(finterp,climCart); colormap gray; colorbar;
