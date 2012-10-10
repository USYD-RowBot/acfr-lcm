function Points = pairwise_pointcloud(TheJournal,est,srcdir,TheConfig);
%function Points = pairwise_pointcloud(TheJournal,est,srcdir,TheConfig);
%
%  INPUTS:
%    TheJournal estimate data structure
%    est        {'Ekf','Eif'}
%    srcdir     full path name of archive directory
%    TheConfig  configuration data structure
%
%  OUTPUT:
%    Points     a point-cloud data structure
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    05-09-2005      rme         Created and written.
%    01-13-2006      rme         Updated help file.

x_vc = TheConfig.SensorXform.PXF.x_vs;
K    = TheConfig.Calib.K;

Nf   = TheJournal.Index.Nf;
Xp_i = TheJournal.Index.Xp_i; % pose index

% find all camera pairs
[Fni,Fnj] = find(TheJournal.Links.vlinks==1);

% init Points structure
Tmp.Xci    = zeros(3,0);
Tmp.Xl     = zeros(3,0);
Tmp.alphaN = zeros(1,0);
Tmp.betaN  = zeros(1,0);
Tmp.gammaN = zeros(1,0);
Tmp.alphaS = zeros(1,0);
Tmp.betaS  = zeros(1,0);
Tmp.gammaS = zeros(1,0);
Points(Nf,Nf).Van     = Tmp;
Points(Nf,Nf).Twoview = Tmp;
Points(Nf,Nf).z_van   = zeros(5,0);
Points(Nf,Nf).z_meas  = zeros(5,0);
Points(Nf,Nf).x_cjci  = zeros(6,0);
Points(Nf,Nf).bmag    = zeros(1,0);

% construct pairwise point cloud
Npairs = length(Fni);
for k=1:Npairs;
  % print to screen
  if mod(k,10)==0;
    tmp = whos('Points');
    mb = tmp.bytes/1024^2;
    fprintf('%d of %d   %.2fMB\n',k,Npairs,mb);
  end;
  % save a copy
  %if mod(k,1000)==0;
  %  save([tempdir,'Points.mat'],'Points');
  %end;
  
  % feature number
  fni = Fni(k);
  fnj = Fnj(k);
  
  % image number
  imgnumi = TheJournal.Index.featureLUT(fni);
  imgnumj = TheJournal.Index.featureLUT(fnj);
  
  % load Features data structure
  Featuresi = loadFeatures([srcdir,'/../featdat'],imgnumi);
  Featuresj = loadFeatures([srcdir,'/../featdat'],imgnumj);

  % load Inliers data structure
  inFile = sprintf('%s/corrset/Inliers-%04d-%04d',srcdir,imgnumi,imgnumj);
  load(inFile);
  
  % pose index of delayed-states
  Xpi = TheJournal.Index.Xf_ii{fni}(Xp_i);
  Xpj = TheJournal.Index.Xf_ii{fnj}(Xp_i);
  
  % vehicle pose
  x_lvi = TheJournal.(est).mu(Xpi);
  x_lvj = TheJournal.(est).mu(Xpj);
  
  % relative camera pose from VAN
  x_cjci = relative_sensor_pose(x_lvj,x_lvi,x_vc);
  R_van = rotxyz(x_cjci(4:6));
  t_van = x_cjci(1:3);
  b    = trans2dm(t_van);
  azel = b(1:2);
  bmag = b(3);
  b_van = t_van/bmag;
  z_van = [azel; x_cjci(4:6)];
  
  % relative camera pose from two-view
  b_2view = dm2trans([Inliers.z(1:2); 1]);
  R_2view = rotxyz(Inliers.z(3:5));
  
  % xform to put points back in world frame
  x_lci = head2tail(x_lvi,x_vc);
  H_lci = [rotxyz(x_lci(4:6)), x_lci(1:3); 0 0 0 1];

  
  % correspondence set
  uci = [Featuresi.Zernike.uc(Inliers.iselZernike1); ...
	 Featuresi.Sift.uc(Inliers.iselSift1)];
  vci = [Featuresi.Zernike.vc(Inliers.iselZernike1); ...
	 Featuresi.Sift.vc(Inliers.iselSift1)];
  ucj = [Featuresj.Zernike.uc(Inliers.iselZernike2); ...
	 Featuresj.Sift.uc(Inliers.iselSift2)];
  vcj = [Featuresj.Zernike.vc(Inliers.iselZernike2); ...
	 Featuresj.Sift.vc(Inliers.iselSift2)];
  
  if length(uci) > 0;
    % triangulate the correspondence set according to VAN
    [Tmp.Xci,Tmp.alphaN,Tmp.betaN,Tmp.gammaN] = triangulate(R_van,b_van,uci,vci,ucj,vcj,K);
    % set scale according to baseline
    Tmp.alphaS = bmag * Tmp.alphaN;
    Tmp.betaS  = bmag * Tmp.betaN;
    Tmp.gammaS = bmag * Tmp.gammaN;
    % xfrom back into local-level frame    
    Xci = Tmp.Xci*bmag;
    Xl = H_lci*[Xci; ones(1,size(Xci,2))];
    Tmp.Xl = Xl(1:3,:);
    Points(fni,fnj).Van = Tmp;
    
    % triangulate the correspondence set according to two-view
    [Tmp.Xci,Tmp.alphaN,Tmp.betaN,Tmp.gammaN] = triangulate(R_2view,b_2view,uci,vci,ucj,vcj,K);
    % set scale according to baseline
    Tmp.alphaS = bmag * Tmp.alphaN;
    Tmp.betaS  = bmag * Tmp.betaN;
    Tmp.gammaS = bmag * Tmp.gammaN;
    % xfrom back into local-level frame    
    Xci = Tmp.Xci*bmag;
    Xl = H_lci*[Xci; ones(1,size(Xci,2))];
    Tmp.Xl = Xl(1:3,:);
    Points(fni,fnj).Twoview = Tmp;

    % stuff misc
    Points(fni,fnj).x_cjci = x_cjci;     % VAN est of relative camera pose
    Points(fni,fnj).bmag   = bmag;       % baseline magnitude according to VAN
    Points(fni,fnj).z_meas = Inliers.z;  % measured cam meas
    Points(fni,fnj).z_van  = z_van;      % predicted cam meas according to VAN est
  end;
end;
save([tempdir,'Points.mat'],'Points');
