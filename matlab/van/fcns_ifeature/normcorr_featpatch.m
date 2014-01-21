function [patchmat,sel] = normcorr_featpatch(Iraw,u_sub,v_sub,tf_rm_distortion,K,R,w)
%function [patchmat,sel] = normcorr_featpatch(Iraw,u_sub,v_sub,tf_rm_distortion,K,R,w)
%  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-27-2003      rme         Created. Loosely based upon Oscar Pizarros 
%                                patch_extract function.
%    08-28-2003      rme         Implemented the different feature patch
%                                extraction methods A,B,C,D. Method C was
%                                found to be the fastest.

% METHOD A is based upon using imtransform to warp the entire raw image based upon
% the infinite homography as well as radial distortion correction.  The
% warped image is then sampled using tformarray to get the desired
% feature patches.  While this method is second fastest, the patch intensities are
% not as accurate because of the double sampling.
  
% METHOD D uses imtransform to warp only a small patch from the raw image
% to get the desired warped feature patch.  The is repeated for each
% feature and is very slow, though the patch intensity values are more
% accurate because of the single sampling.
  
% METHOD B is an attempt to accomplish the same result as METHOD D, but
% by using tformarray directly to speed up the processing.  This method
% is however, not much faster than METHOD D.
  
% METHOD C is the fastest.  METHOD C follows the same princle as METHOD B
% but uses tformarray in a much more computationaly efficient way.  The
% large speed up is mainly due to only having to compute the tform
% tf_tformarray once at the begining of the loop.  This is the method
% which this function implements.

  
DEBUG = false;
FIGURES = false;
interp = 'linear'; %{'nearest','linear','cubic'}

if DEBUG
  [tA,tB,tC,tD] = deal(0);
end

% w is half size of feature patch (i.e. feature patch is (2w+1)x(2w+1))
  
[nr,nc] = size(Iraw);

% worst case rotation is 45 degrees, therefore select features around
% image border within this constraint
b = ceil(sqrt(2)*w);
sel = find((u_sub>b) & (u_sub<nc-b) & (v_sub>b) & (v_sub<nr-b));
%sel = find((u_sub+1>b+1) & (u_sub+1<nc-b) & (v_sub+1>b+1) & (v_sub+1<nr-b));
u_sub = u_sub(sel); v_sub = v_sub(sel);

% pre-allocate memory to hold feature vectors
Nf = length(sel);
patchmat = zeros((2*w+1)^2,Nf);


% compose homography at infinity based upon camera pose
Hinf = homog_Hinf(K,R);


% create a matlab tform structure for Hinf
% note matlab uses post vs. pre multiply
tf_Hinf = maketform('projective',Hinf');


% create a composite matlab tfrom structure which implements
% the radial distortion correction followed by the warping of the 
% homography at infinity
tf_comp = maketform('composite',tf_Hinf,tf_rm_distortion);

% raw image input coordinates
udata = [1 nc]-1;
vdata = [1 nr]-1;

if DEBUG % used for METHOD A
  tic;
  % warp the raw image by the composite tform
  [Iwarp,xdata_warp,ydata_warp] = imtransform(Iraw,tf_comp,interp, ...
					      'udata',udata,'vdata',vdata);
  tA = toc;
  fprintf('%s warp took %gs\n',interp,tA);
else
  inbounds = [udata',vdata'];
  outbounds = findbounds(tf_comp,inbounds);
  xdata_warp = outbounds(:,1)';
  ydata_warp = outbounds(:,2)';
end

% calculate location of feature points in warped image
tmp = tformfwd([u_sub,v_sub],tf_comp);
uw = tmp(:,1);
vw = tmp(:,2);

% convert warped feature coordinates (uw,vw) to index correctly into the
% matlab warped image array
ui = uw-xdata_warp(1)+1;
vi = vw-ydata_warp(1)+1;


if FIGURES
  figure(100);
  imshow(xdata_warp,ydata_warp,Iwarp);
  hold on;
  plot(uw,vw,'y+');  
end
  
if DEBUG % used for METHOD D
  % round subpixel (u,v) coordinates to nearest integer
  u_int = round(u_sub);
  v_int = round(v_sub);
end

% create a resampler structure
R = makeresampler(interp,'fill');

% create a tform which works with image array coordinates
tf_tformarrayC = make_2d_tformarray_tform(tf_comp,[nr nc],udata,vdata,xdata_warp,ydata_warp);

% for each feature, grab it's associated warped feature patch
% by resampling the warped image using R
fpind = -w:w;  % feature patch index
for ii=1:Nf
  if DEBUG
    % FEATURE PATCH METHOD A
    tic;
    [u_fp,v_fp] = meshgrid(ui(ii)+fpind,vi(ii)+fpind);
    tmap_b = cat(3,u_fp,v_fp);
    xdataA = [min(u_fp(:)) max(u_fp(:))]+xdata_warp(1)-1;
    ydataA = [min(v_fp(:)) max(v_fp(:))]+ydata_warp(1)-1;
    featpatchA = tformarray(Iwarp,[],R,[2 1],[1 2],[],tmap_b,0);
    tA = tA + toc;
  
    % FEATURE PATCH METHOD B
    tic;
    % input image coordinates
    udataB = [1 nc]-1;
    vdataB = [1 nr]-1;  
    % feature patch coordinates in the warped image
    xdataB = [uw(ii)-w uw(ii)+w];
    ydataB = [vw(ii)-w vw(ii)+w];
    tsize_b = [2*w+1, 2*w+1];
    tf_tformarrayB = make_2d_tformarray_tform(tf_comp,[nr nc],udataB,vdataB,xdataB,ydataB);  
    featpatchB = tformarray(Iraw,tf_tformarrayB,R,[2 1],[2 1],tsize_b,[],0);
    tB = tB + toc;
    
    % FEATURE PATCH METHOD D
    tic;
    % grab a region from the raw image associated with the feature point
    % note that 1 is added for corrected matlab indexing since i define the
    % top left corner to be (0,0) while matlab defines it to be (1,1)
    rawind = -b:b; % raw patch index    
    rawpatch = Iraw(v_int(ii)+1+rawind,u_int(ii)+1+rawind);
    % raw patch coordinates
    udataD = [u_int(ii)-b u_int(ii)+b];
    vdataD = [v_int(ii)-b v_int(ii)+b];
    % feature patch coordinates in the warped image
    xdataD = [uw(ii)-w uw(ii)+w];
    ydataD = [vw(ii)-w vw(ii)+w];
    % warped feature patch
    [featpatchD,xdata_out,ydata_out] = imtransform(rawpatch,tf_comp,R, ...
						   'udata',udataD,'vdata',vdataD, ...
						   'xdata',xdataD,'ydata',ydataD); 
    difx = round(xdata_out(2) - xdataD(2));
    dify = round(ydata_out(2) - ydataD(2));
    if difx > 0
      featpatchD = featpatchD(:,1:end-difx);
    end
    if dify > 0
      featpatchD = featpatchD(1:end-dify,:);
    end
    tD = tD + toc;
  end % if DEBUG

  % FEATURE PATCH METHOD C
  if DEBUG; tic; end  
  % feature patch coordinates in the warped image
  xdataC = [uw(ii)-w uw(ii)+w];
  ydataC = [vw(ii)-w vw(ii)+w];
  % feature patch *matlab array* coordinates in the warped image
  [u_fp,v_fp] = meshgrid(ui(ii)+fpind,vi(ii)+fpind);
  tmap_b = cat(3,u_fp,v_fp);  
  featpatchC = tformarray(Iraw,tf_tformarrayC,R,[2 1],[1 2],[],tmap_b,0);
  if DEBUG; tC = tC + toc; end

  
  % NORMALIZE AND STORE FEATURE PATCH
  % vectorize image patch
  patchvec = double(featpatchC(:));
  % de-mean and normalize
  patchvec = patchvec - mean(patchvec);
  energy = patchvec'*patchvec;
  patchvec = patchvec/sqrt(energy);
  % store feature vector
  patchmat(:,ii) = patchvec;

  
  if FIGURES
    figure(101);
    subplot(241);
    imagesc(udataD,vdataD,rawpatch);
    colormap gray; axis equal; set(gca,'ydir','reverse');
    title('rawpatch');
    subplot(245);
    imagesc(xdataA,ydataA,featpatchA);
    colormap gray; axis equal; set(gca,'ydir','reverse');
    title('featpatchA');
    subplot(246);
    imagesc(xdataB,ydataB,featpatchB);
    colormap gray; axis equal; set(gca,'ydir','reverse');
    title('featpatchB');
    subplot(247);
    imagesc(xdataC,ydataC,featpatchC); 
    colormap gray; axis equal; set(gca,'ydir','reverse');
    title('featpatchC');
    subplot(248);
    imagesc(xdataD,ydataD,featpatchD); 
    colormap gray; axis equal; set(gca,'ydir','reverse');
    title('featpatchD');    
    keyboard;
  end

end % for ii=1:Nf

if DEBUG
  % display total computation time for each method
  disp('A       B        C       D');
  disp([tA tB tC tD]);
end
