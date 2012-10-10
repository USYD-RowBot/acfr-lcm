function data = data_association_comparison(TJ,TheConfig);

global TheJournal;

TheJournal = TJ;
  
fnj  = TheJournal.Index.fn;
Xp_i = TheJournal.Index.Xp_i;
Xfj_i = TheJournal.Index.Xf_ii{fnj}(Xp_i);
[data.fni,data.normci,data.normseif, data.mb, ...
 data.Trace.eif,data.Trace.ci,data.Trace.seif, ...
 data.Det.eif,data.Det.ci,data.Det.seif] = deal(zeros(fnj-1,1));
[data.Peif,data.Pci,data.Pseif, ...
 data.Veif,data.Vci,data.Vseif, ...
 data.Deif,data.Dci,data.Dseif] = deal(zeros(6,6,fnj-1));

if isempty(TheJournal.Ekf.Sigma)
  % fake the EKF covariance matrix so i can use existing functions
  TheJournal.Ekf.mu = TheJournal.Eif.mu;
  TheJournal.Ekf.Sigma = TheJournal.Eif.Sigma; % block diag
  TheJournal.Ekf.Sigma(:,Xfj_i) = TheJournal.Eif.SigmaCol;
  TheJournal.Ekf.Sigma(Xfj_i,:) = TheJournal.Eif.SigmaCol';
end;

for fni=1:fnj-1;
  disp(fni);
  
  data.fni(fni) = fni;
  
  % eif covariance
  TheConfig.Estimator.inferenceEif = off;
  data.Peif(:,:,fni) = relpose_uncertainty(TheJournal,fni,fnj,TheConfig);
  
  % eif DA bounds
  TheConfig.Estimator.inferenceEif = on;
  TheConfig.Estimator.useSeifsDA = off;
  data.Pci(:,:,fni) = relpose_uncertainty(TheJournal,fni,fnj,TheConfig);
  
  % eif seifs
  TheConfig.Estimator.inferenceEif = on;
  TheConfig.Estimator.useSeifsDA = on;
  data.Pseif(:,:,fni) = relpose_uncertainty(TheJournal,fni,fnj,TheConfig);

  % indices of the shortest path
  mplus_i = markov_blanket(TheJournal.Links.fgraph,fni);
  data.mb(fni) = length(mplus_i);
  
  % metrics
  [U,S,V] = svd(data.Peif(:,:,fni));  [data.Veif(:,:,fni),data.Deif(:,:,fni)] = deal(U,S);
  [U,S,V] = svd(data.Pseif(:,:,fni)); [data.Vseif(:,:,fni),data.Dseif(:,:,fni)] = deal(U,S);
  [U,S,V] = svd(data.Pci(:,:,fni));   [data.Vci(:,:,fni),data.Dci(:,:,fni)] = deal(U,S);

  data.normci(fni)   = norm(eye(6)-data.Veif(:,:,fni)'*data.Vci(:,:,fni));
  data.normseif(fni) = norm(eye(6)-data.Veif(:,:,fni)'*data.Vseif(:,:,fni));
  %data.Ici(:,:,fni)   = data.Peif(:,:,fni)*spdinverse(data.Pci(:,:,fni));
  %data.normci(fni)    = norm(squeeze(data.Ici(:,:,fni)),'inf');
  %data.Iseif(:,:,fni) = data.Peif(:,:,fni)*spdinverse(data.Pseif(:,:,fni));
  %data.normseif(fni)  = norm(squeeze(data.Iseif(:,:,fni)),'inf');
  
  data.Trace.eif(fni)  = trace(squeeze(data.Peif(:,:,fni)));
  data.Trace.ci(fni)   = trace(squeeze(data.Pci(:,:,fni)));
  data.Trace.seif(fni) = trace(squeeze(data.Pseif(:,:,fni)));
  data.Det.eif(fni)  = det(squeeze(data.Peif(:,:,fni)))^(1/6);
  data.Det.ci(fni)   = det(squeeze(data.Pci(:,:,fni)))^(1/6);
  data.Det.seif(fni) = det(squeeze(data.Pseif(:,:,fni)))^(1/6);
end;

TheJournal = TJ;


%================================================================
function P_cjci = relpose_uncertainty(TheJournal,fni,fnj,TheConfig);
% extract pose information from our state estimate
[x_lvi,x_lvj,x_vc,Sigma] = extract_poses(fni,fnj,TheConfig);
	      

[x_cjci,J_cjci] = relative_sensor_pose(x_lvj,x_lvi,x_vc);
J_cjci = J_cjci(:,[7:12,1:6,13:18]); % switch to match order of Sigma

% 1st order relative pose uncertainty
P_cjci = J_cjci*Sigma*J_cjci';

