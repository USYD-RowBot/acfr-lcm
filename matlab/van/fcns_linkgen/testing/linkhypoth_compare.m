function [link_t,fni,fnj] = linkhypoth_compare(link_t,TheJournal,TheConfig)
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

POVERLAP = 0.25;  % percent image overlap [0 1]
THRESH = 0.99;   % confidence in overlap [0 1]

% static camera to vehicle xform
x_vc = TheConfig.SensorXform.PXF.x_vs;
  
Ni = find(TheJournal.Index.featureLUT > 0); 
Ni = Ni(1);              % index of first camera feature
Nf = TheJournal.Index.Nf; % index of last camera feature

% initialize and grow links matrices to current number of images
link_t.plinks(1:Nf,1:Nf) = 0;
link_t.vlinks(end:Nf,end:Nf) = 0;

% precompute some usefull terms
SigmaFull = full(TheJournal.Eif.Lambda)^-1;
LambdaTriDiag = tridiag(TheJournal.Eif.Lambda,TheJournal.Index.Nv);
SigmaTriDiag = full(LambdaTriDiag^-1);
pathlen = zeros(size(link_t.fgraph));

% compute Euclidian horizontal distance between camera centers
% and use it as a metric for hypothesizing overlapping image pairs
for ii=Ni:Nf
  % index of vehicle state fni
  Xf_i_full = TheJournal.Index.Xf_ii{ii};   % full vehicle state index
  Xf_i = Xf_i_full(TheJournal.Index.Xp_i);  % vehicle pose elements index
  xi_lv = TheJournal.Ekf.mu(Xf_i);                % vehicle pose
  [xi_lc,Ji_plus] = head2tail(xi_lv,x_vc); % camera pose in local-level frame

  for jj=ii+1:Nf
    if link_t.vlinks(ii,jj) ~= 0 || ii < 10
      % skip this image pair all together since either:
      % 1) vlinks < 0 previous reg attempt failed
      % 2) vlinks = 1 image pair has already been successfully registered
      continue;
    end
    
    % index of vehicle pose fnj
    Xf_j_full = TheJournal.Index.Xf_ii{jj};   % full vehicle state index
    Xf_j  = Xf_j_full(TheJournal.Index.Xp_i); % vehicle pose elements index
    xj_lv = TheJournal.Ekf.mu(Xf_j);                % vehicle pose
    [xj_lc,Jj_plus] = head2tail(xj_lv,x_vc); % camera pose in local-level frame
    
    % Euclidean horizontal distance between camera centers
    [d,J_ij] = edist(xi_lc(1:2),xj_lc(1:2));
    jrnl_t.d(ii,jj) = d;
    
    % total Jacobian of Euclidean distance, d, w.r.t vehicle poses xi_lv & xj_lv
    J = [J_ij(1:2)*Ji_plus(1:2,1:6), J_ij(3:4)*Jj_plus(1:2,1:6)];
    
    % max altitude for camera pair
    %alt = max([TheBathy(ii).alt; TheBathy(jj).alt]);
    alt = 1.4;
    jrnl_t.alt(ii,jj) = alt;

    % full covariance
    n = 1;
    jrnl_t.method{n} = 'full_covariance';
    jrnl_t.Sigma{n,ii,jj} = SigmaFull([Xf_i,Xf_j],[Xf_i,Xf_j]);
    
    % conditional covariance
    n = n+1;
    jrnl_t.method{n} = 'conditional_covariance';
    jrnl_t.Sigma{n,ii,jj} = conditional_covariance(TheJournal.Eif.Lambda,Xf_i_full,Xf_j_full);

    % markov blanket
    n = n+1;
    jrnl_t.method{n} = 'condcov_markov_blanket';
    jrnl_t.Sigma{n,ii,jj} = condcov_mb(TheJournal.Eif.Lambda,Xf_i_full,Xf_j_full);
    
    % markov blanket thrun
    n = n+1;
    jrnl_t.method{n} = 'condcov_markov_blanket_thrun';
    [jrnl_t.Sigma{n,ii,jj},jrnl_t.pathlen(ii,jj)] = condcov_mb_thrun(TheJournal.Eif.Lambda,Xf_i_full,Xf_j_full,link_t.fgraph,ii,jj,TheJournal.Index);
    
    % positive definite
    n = n+1;
    jrnl_t.method{n} = 'positive definite 1';
    jrnl_t.Sigma{n,ii,jj} = posdef_approx1(TheJournal.Eif.Lambda,Xf_i_full,Xf_j_full);
    
    % positive definite
    n = n+1;
    jrnl_t.method{n} = 'positive definite 2';
    jrnl_t.Sigma{n,ii,jj} = posdef_approx2(TheJournal.Eif.Lambda,Xf_i_full,Xf_j_full);    
    
    fprintf('(%d,%d)/(%d,%d)\td=%.2f\tpathlen=%d\n',ii,jj, ...
	    TheJournal.Index.featureLUT(ii),TheJournal.Index.featureLUT(jj),d,jrnl_t.pathlen(ii,jj));
    fprintf('normdet\t\td_sigma\t\td_sigma_ratio\tp\tmethod\n');
    fprintf('-------------------------------------------------------------------\n');
    for n=1:length(jrnl_t.method)
      % normalized determinant
      jrnl_t.normdet(n,ii,jj) = det(jrnl_t.Sigma{n,ii,jj})^(1/6);
      
      % 1st oder std of Euclidean distance d
      jrnl_t.d_sigma(n,ii,jj) = sqrt(J*jrnl_t.Sigma{n,ii,jj}*J');
      
      % ratio of sigma to true
      jrnl_t.d_sigma_ratio(n,ii,jj) = jrnl_t.d_sigma(1,ii,jj)/jrnl_t.d_sigma(n,ii,jj);
    
      % probability that the Euclidean distance d is less than the altitude
      jrnl_t.p(n,ii,jj) = normcdf((1-POVERLAP)*alt,d,jrnl_t.d_sigma(n,ii,jj));
      
      fprintf('%.2g\t\t%.2g\t\t%.2g\t\t%.2g\t%s\n',jrnl_t.normdet(n,ii,jj),jrnl_t.d_sigma(n,ii,jj), ...
	      jrnl_t.d_sigma_ratio(n,ii,jj), jrnl_t.p(n,ii,jj),jrnl_t.method{n});
    end
    fprintf('\n');
    
    keyboard;
  end % for jj

end % for ii

%*************************************************************************************
function Sigma = conditional_covariance(Lambda,Xf_i_full,Xf_j_full)

% full state conditional covariance
xij = [Xf_i_full,Xf_j_full];
Sigma = full(Lambda(xij,xij)^-1);

% pose elements conditional covariance
ii = [1:6,13:18];
Sigma = Sigma(ii,ii);

%*************************************************************************************
function Sigma = condcov_mb(Lambda,Xf_i_full,Xf_j_full)

% indices
xij = [Xf_i_full,Xf_j_full];
Mplus_i = markov_blanket(Lambda,xij);
Sigma = full(Lambda([xij,Mplus_i],[xij,Mplus_i])^-1);

% pose elements conditional covariance
ii = [1:6,13:18];
Sigma = Sigma(ii,ii);

%*************************************************************************************
function [Sigma,pathlen] = condcov_mb_thrun(Lambda,Xf_i_full,Xf_j_full,Amatrix,fni,fnj,index_t)

% indices
xij = [Xf_i_full,Xf_j_full];
Mplus_i = markov_blanket(Lambda,xij);
[pathlen,Path] = dijkstra(Amatrix,fni,fnj);
Mpath_i = [index_t.Xf_ii{Path(2:end-1)}];
Mpath_i = setdiff(Mpath_i,Mplus_i);
Sigma = full(Lambda([xij,Mplus_i,Mpath_i],[xij,Mplus_i,Mpath_i])^-1);

% pose elements conditional covariance
ii = [1:6,13:18];
Sigma = Sigma(ii,ii);

%*************************************************************************************
function Sigma = posdef_approx1(Lambda,Xf_i_full,Xf_j_full)
res = 0.025;

% indices
xij = [Xf_i_full,Xf_j_full];
Mplus_i = markov_blanket(Lambda,xij);
PDproduct = symprod(Lambda(xij,xij)^-1,Lambda(xij,Mplus_i));
[R,p] = chol(PDproduct); p
done = 0;
alpha = 0;
while ~done
  Lambda_star = [Lambda(xij,xij),     Lambda(xij,Mplus_i); ...		 
		 Lambda(Mplus_i,xij), Lambda(Mplus_i,Mplus_i) - alpha*PDproduct];
  [R,p] = chol(Lambda_star);
  if p == 0
    alpha = alpha + res;
  else
    alpha = alpha - res;
    done = 1;
  end
end
alpha
Lambda_star = [Lambda(xij,xij),     Lambda(xij,Mplus_i); ...
	       Lambda(Mplus_i,xij), Lambda(Mplus_i,Mplus_i) - alpha*PDproduct];
Sigma = full(Lambda_star^-1);

% pose elements conditional covariance
ii = [1:6,13:18];
Sigma = Sigma(ii,ii);

%*************************************************************************************
function Sigma = posdef_approx2(Lambda,Xf_i_full,Xf_j_full)
res = 0.005;
  
% indices
xij = [Xf_i_full,Xf_j_full];
Mplus_i = markov_blanket(Lambda,xij);
Mminus_i = 1:length(Lambda);
Mminus_i([xij,Mplus_i]) = [];
done = 0;
alpha = 1;
LambdaTriDiag = tridiag(Lambda(Mminus_i,Mminus_i),12);
SigmaTriDiag = full(LambdaTriDiag^-1);
PDproduct = symprod(SigmaTriDiag,Lambda(Mminus_i,Mplus_i));
while ~done
  Lambda_star = [Lambda(xij,xij),     Lambda(xij,Mplus_i); ...
		 Lambda(Mplus_i,xij), Lambda(Mplus_i,Mplus_i) - alpha*PDproduct];
  [R,p] = chol(Lambda_star);
  if p == 0
    alpha = alpha+res;
  else
    alpha = alpha-res;
    done = 1;
  end
end
alpha
Lambda_star = [Lambda(xij,xij),     Lambda(xij,Mplus_i); ...
	       Lambda(Mplus_i,xij), Lambda(Mplus_i,Mplus_i) - alpha*PDproduct];
Sigma = full(Lambda_star^-1);

% pose elements conditional covariance
ii = [1:6,13:18];
Sigma = Sigma(ii,ii);
