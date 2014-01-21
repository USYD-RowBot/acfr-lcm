function cmatrix = eliminate_nonunique(cmatrix,smatrix1,smatrix2,thresh)


% cmatrix is a [M x N] correspondence matrix:
% M is the number of features in image I
% N is the number of features in image I'
% 
% A nonzero entry in cmatrix(i,j) indicates a possible correspondence
% between features Mi and Nj.
% 
% smatrix1 is a [M x M] self-similarity matrix for features in image I.
  
%cmatrix_orig = cmatrix;
%tic;

cmatrix = private_nonunique(cmatrix,smatrix2,thresh);
cmatrix = private_nonunique(cmatrix',smatrix1,thresh)';

%fprintf('\n%s: done: %f  nnz_orig: %d  nnz_prune: %d\n\n',...
%	mfilename,toc,nnz(cmatrix_orig),nnz(cmatrix));
%figure(1); spy(cmatrix_orig); title('cmatrix orig'); grid on;
%figure(2); spy(cmatrix); title('cmatrix prune'); grid on;
%keyboard;


function cmatrix = private_nonunique(cmatrix,smatrix,thresh)

% cmatrix is organized by [I x I']
% smatrix is self-similarity of features in I'
% thresh is similarity threshold for picking correspondences

for k=find(any(cmatrix,2))'
  % find feature points in I' which correspond to feature k in I
  fsel = find(cmatrix(k,:));  
  Nf = length(fsel);
  
  % local self-similarity matrix compared to threshold
  lsmatrix = smatrix(fsel,fsel) > thresh;
  % remove auto-similarity scores from matrix (i.e. diagonal elements)
  lsmatrix([0:Nf-1]*Nf+[1:Nf]) = false;
  rsel = find(any(lsmatrix,2));
  
  cmatrix(k,fsel(rsel)) = false;
  %figure(1); spy(lsmatrix); title('lsmatrix orig'); grid on;
  while false %any(lsmatrix(:))
    % number of intra-area matches
    Nmatches = sum(lsmatrix,2);
    % find most ambiguous feature
    [Nmax,ii] = max(Nmatches);
    % remove it from the local self-similarity matrix
    lsmatrix(ii,:) = false;
    lsmatrix(:,ii) = false;
    % remove it from the set of candidate features for feature k
    cmatrix(k,fsel(ii)) = false;
    %figure(2); spy(lsmatrix); title('lsmatrix prune'); grid on;
    %keyboard;
  end
  
end


