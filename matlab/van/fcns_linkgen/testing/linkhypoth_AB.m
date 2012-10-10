function [link_t,fni,fnj] = linkhypoth_AB(link_t,TheJournal,TheConfig)
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

Amatrix = TheJournal.Eif.Lambda(1:12:end,1:12:end);
ii = find(Amatrix);
Amatrix(ii) = nan;

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
    if ii < 20 || link_t.vlinks(ii,jj) ~= 0
      % skip this image pair altogether since either:
      % 1) vlinks < 0 previous reg attempt failed
      % 2) vlinks = 1 image pair has already been successfully registered
      continue;
    end
    
    disp([ii,jj])
    
    % vehicle pose, covariance, and cross-covariance in local-level frame
    Xf_j_full = TheJournal.Index.Xf_ii{jj};   % full vehicle state index
    Xf_j  = Xf_j_full(TheJournal.Index.Xp_i); % pose elements index
    xj_lv = TheJournal.Ekf.mu(Xf_j);
    
    % full covariance
    SigmaA = TheJournal.Ekf.Sigma([Xf_i,Xf_j],[Xf_i,Xf_j]);
    RhoA = rhomatrix(SigmaA)
    % conditional covariance
    SigmaB = full(TheJournal.Eif.Lambda([Xf_i_full,Xf_j_full],[Xf_i_full,Xf_j_full])^-1);
    xii = TheJournal.Index.Xp_i;
    xji = xii + TheJournal.Index.Nv;
    SigmaB = SigmaB([xii,xji],[xii,xji]);
    RhoB = rhomatrix(SigmaB);
    % conservative estimate
    m_plus = markov_blanket(TheJournal.Eif.Lambda,[Xf_i_full,Xf_j_full]);
    m_minus = 1:TheJournal.Index.Naug;
    m_minus([Xf_i_full,Xf_j_full,m_plus]) = [];
    tmp = [1:length(m_plus)]+24;
    LambdaC(1:24,1:24) = TheJournal.Eif.Lambda([Xf_i_full,Xf_j_full],[Xf_i_full,Xf_j_full]);
    LambdaC(1:24,tmp) = TheJournal.Eif.Lambda([Xf_i_full,Xf_j_full],m_plus);
    LambdaC(tmp,1:24) = TheJournal.Eif.Lambda(m_plus,[Xf_i_full,Xf_j_full]);
    switch 1
    case 0
     LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_plus,m_plus);
    case 1
     if ~any(any(TheJournal.Eif.Lambda(Xf_i_full,m_plus) & TheJournal.Eif.Lambda(Xf_j_full,m_plus)))
       disp('add extra');
       [D,dpath] = dijkstra(Amatrix,ii,jj);
       dpath
       m_path = [TheJournal.Index.Xf_ii{dpath(2:end-1)}];
       m_all = unique([m_path,m_plus]);
     else
       m_all = m_plus;
     end
     tmp = [1:length(m_all)]+24;
     LambdaC(1:24,tmp) = TheJournal.Eif.Lambda([Xf_i_full,Xf_j_full],m_all);
     LambdaC(tmp,1:24) = TheJournal.Eif.Lambda(m_all,[Xf_i_full,Xf_j_full]);     
     LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_all,m_all);
    case 2
     LambdaC(tmp,tmp) = 1.05*LambdaC(tmp,1:24)*LambdaC(1:24,1:24)^-1*LambdaC(1:24,tmp);
    case 3
     alpha = 0;
     done = 0;
     delta = 0.01;
     LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_plus,m_plus);
     while ~done
       [R,p] = chol(LambdaC);       
       if p == 0
	 alpha = alpha + delta;
	 LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_plus,m_plus) ...
	     - alpha*LambdaC(tmp,1:24)*LambdaC(1:24,1:24)^-1*LambdaC(1:24,tmp);
       else
	 done = 1;	 
	 alpha = alpha - delta
	 LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_plus,m_plus) ...
	     - alpha*LambdaC(tmp,1:24)*LambdaC(1:24,1:24)^-1*LambdaC(1:24,tmp);	 
       end
     end
    case 4
     alpha = 1;
     LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_plus,m_plus) ...
	 - alpha*LambdaC(tmp,1:24)*LambdaC(1:24,1:24)^-1*LambdaC(1:24,tmp);    
    case 5
     LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_plus,m_plus) ...
	 - TheJournal.Eif.Lambda(m_plus,m_minus) * ...
	   diag(diag(TheJournal.Eif.Lambda(m_minus,m_minus)).^-1) * ...
	   TheJournal.Eif.Lambda(m_minus,m_plus);
    case 6
     kk = find(any(TheJournal.Eif.Lambda(m_plus,m_minus)));
     LambdaC(tmp,tmp) = TheJournal.Eif.Lambda(m_plus,m_plus) ...
	 - TheJournal.Eif.Lambda(m_plus,m_minus(kk)) * ...
	   TheJournal.Eif.Lambda(m_minus(kk),m_minus(kk))^-1 * ...
	   TheJournal.Eif.Lambda(m_minus(kk),m_plus);
    end
	               
    SigmaC = full(LambdaC)^-1;
    SigmaC = SigmaC([xii,xji],[xii,xji]);
    RhoC = rhomatrix(SigmaC)
    
    
    % camera pose in local-level frame
    [xj_lc,Jj_plus] = head2tail(xj_lv,x_vc);
    
    % horizontal distance between camera centers
    [d,J_ij] = edist(xi_lc(1:2),xj_lc(1:2));
    
    % total Jacobian of euclidean distance d w.r.t vehicle poses xi_lv xj_lv
    J = [J_ij(1:2)*Ji_plus(1:2,1:6), J_ij(3:4)*Jj_plus(1:2,1:6)];
    
    % determinant
    detrm.A = det(SigmaA)^(1/length(SigmaA));
    detrm.B = det(SigmaB)^(1/length(SigmaB));
    detrm.C = det(SigmaC)^(1/length(SigmaC))
    
    % 1st order covariance of distance d
    Cov_d.A = J*SigmaA*J';
    Cov_d.B = J*SigmaB*J';
    Cov_d.C = J*SigmaC*J'

    sigma_d.A = sqrt(Cov_d.A);
    sigma_d.B = sqrt(Cov_d.B);
    sigma_d.C = sqrt(Cov_d.C)
    sigma_d.A/sigma_d.C
    
    % max altitude for camera pair
    %alt = max([TheBathy(ii).alt; TheBathy(jj).alt]);
    alt = 1.4;
    
    % probability that the euclidean distance d is less than the altitude
    p.A = normcdf((1-POVERLAP)*alt,d,sigma_d.A);
    p.B = normcdf((1-POVERLAP)*alt,d,sigma_d.B);
    p.C = normcdf((1-POVERLAP)*alt,d,sigma_d.C)
    link_t.plinks(ii,jj) = p.A;
    
    keyboard;
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
