function [link_t,fni,fnj] = linkhypoth_eifB(link_t,TheJournal,TheConfig)
% INPUT:
% link_t contains fields plinks and vlinks
% plinks is an upper triangular [Nf x Nf] array of 0's and 1's where a 1
% represents a hypothesized correspondence between feature i and j
% vlinks is also an upper triangular [Nf x Nf] array.
%   0 means a correspondence between i and j has not been tried
%   1 means a correspondence between i and j has been established
%  -N means a correspondence between i and j has been tried N times and
%     has failed each time.
% TheJournal is augmented state structure
% x_vc is [6x1] camera to vehicle pose vector
%
% OUTPUT:
% link_t
% fni,fnj feature indexes of hypothesized overlapping image pairs
%         note that fni < fnj always
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    02-27-2004      rme         Created from linkhypoth_manual.m
%    03-30-2004      rme         Modified to calculate 1st order
%                                covariance associated with Euclidean
%                                distance.  Also changed to use
%                                TheBathy altitude information.
%    04-08-2004      rme         Updated input/output arguments.
%    09-09-2004      rme         Updated to extract the Xp_i pose elements from Xf_ii
%                                Limited max number of link candidates


x_vc = TheConfig.SensorXform.PXF.x_vs;
  
POVERLAP = 0.25;  % percent image overlap [0 1]
THRESH = 0.99;   % confidence in overlap [0 1]

Ni = find(TheJournal.Index.featureLUT > 0); 
Ni = Ni(1);            % index of first camera feature
Nf = TheJournal.Index.Nf; % index of last camera feature

% initialize and grow links matrices to current number of images
link_t.plinks(1:Nf,1:Nf) = 0;
link_t.vlinks(end:Nf,end:Nf) = 0;

% compute Euclidian horizontal distance between camera centers
% and use it as a metric for hypothesizing overlapping image pairs
for ii=Ni:Nf
  % vehicle pose and covariance in local-level frame
  Xf_i_full = TheJournal.Index.Xf_ii{ii};  % full vehicle state index
  Xf_i = Xf_i_full(TheJournal.Index.Xp_i); % pose elements index
  xi_lv = TheJournal.Ekf.mu(Xf_i);

  % camera pose in local-level frame
  [xi_lc,Ji_plus] = head2tail(xi_lv,x_vc);

  for jj=ii+1:Nf
    if link_t.vlinks(ii,jj) ~= 0
      % skip this image pair altogether since either:
      % 1) vlinks < 0 previous reg attempt failed
      % 2) vlinks = 1 image pair has already been successfully registered
      continue;
    end
    
    % vehicle pose, covariance, and cross-covariance in local-level frame
    Xf_j_full = TheJournal.Index.Xf_ii{jj};   % full vehicle state index
    Xf_j  = Xf_j_full(TheJournal.Index.Xp_i); % pose elements index
    xj_lv = TheJournal.Ekf.mu(Xf_j);
    
    % conditional covariance
    Sigma = TheJournal.Eif.Lambda([Xf_i_full,Xf_j_full],[Xf_i_full,Xf_j_full])^-1;
    xii = TheJournal.Index.Xp_i;
    xji = xii + TheJournal.Index.Nv;
    Cov_xi_lv = Sigma(xii,xii);
    Cov_xj_lv = TheJournal.Ekf.Sigma(xji,xji);
    Cov_xixj_lv = TheJournal.Ekf.Sigma(xii,xji);
    
    % camera pose in local-level frame
    [xj_lc,Jj_plus] = head2tail(xj_lv,x_vc);
    
    % horizontal distance between camera centers
    [d,J_ij] = edist(xi_lc(1:2),xj_lc(1:2));
    
    % total Jacobian of euclidean distance d w.r.t vehicle poses xi_lv xj_lv
    J = [J_ij(1:2)*Ji_plus(1:2,1:6), J_ij(3:4)*Jj_plus(1:2,1:6)];
    
    % 1st order covariance of distance d
    Cov_p = [Cov_xi_lv,    Cov_xixj_lv;
	     Cov_xixj_lv', Cov_xj_lv];
    Cov_d = J*Cov_p*J';
    sigma_d = sqrt(Cov_d);
    
    % max altitude for camera pair
    %alt = max([TheBathy(ii).alt; TheBathy(jj).alt]);
    alt = 1.4;
    
    % probability that the euclidean distance d is less than the altitude
    p = normcdf((1-POVERLAP)*alt,d,sigma_d);
    link_t.plinks(ii,jj) = p;

  end % for jj

end % for ii

% upper triangular matrix of hypothesized overlaping image pairs
testlinks = ((link_t.plinks > THRESH) - abs(link_t.vlinks)) == 1;
% state vector feature numbers.
[fni,fnj,val] = find(testlinks); % i < j always because testlinks is upper triangular

% sort returns ascending order
[val,ii] = sort(val);
% put in descending order
val = val(end:-1:1);
ii  = ii(end:-1:1);
fni = fni(ii);
fnj = fnj(ii);

% keep only the K strongest candidates
kk = min(length(fni),inf);%;TheConfig.Estimator.max_link_hypoth);
fni = fni(1:kk);
fnj = fnj(1:kk);

%disp('testlinks = ');
%disp([fni fnj]);

if false
figure(2000); imagesc(link_t.plinks,[0 1]); 
colormap jet; colorbar; axis square; title('plinks probability');

figure(2001); imagesc(link_t.vlinks,[-1 1]);
colormap gray; colorbar; axis square; title('vlinks');


tic;
figure(2003);
subplot(1,2,1);
plot_corrcoef(TheJournal.Ekf.Sigma,1);
%spy(TheJournal.Ekf.Sigma);
xlabel('Sigma');
subplot(1,2,2);
plot_corrcoef(TheJournal.Eif.Lambda,1);
%spy(TheJournal.Eif.Lambda); axis equal tight;
xlabel('Lambda');
etime = toc;
fprintf('takes %.2f seconds to display normalized corr/info matrix\n',etime);
end
