function sel = zernikeUniqueNormCorr(psel1,psel2,A1_nm,A2_nm,Zbasis,cmatrix,thresh);
%   INPUTS:
%   psel1,psel2: are [L x 1] vectors defining putative feature correspondences
%
%       cmatrix: is a [M x N] correspondence matrix where M is the number of
%                features in image I1 and N is the number of features in image I2.  A
%                nonzero entry in cmatrix(i,j) indicates a possible pose correspondence
%                between features Mi and Nj.
% 
%   A1_nm,A2_nm: are the [K x M] and [K x N] Zernike moments
%        Zbasis: is the Zernike basis functions data structure
%        thresh: is the normalized correlation threshold
%   OUTPUT:
%           sel: is a vector containing the indexes of unique putative matches

% allocate self-similarity matrices
[M,N] = size(cmatrix);
smatrix1(M,M,length(psel1)^2);
smatrix2(N,N,length(psel1)^2);
% self-similarity matrices evaluated at putative correspondences only
smatrix1(psel1,psel1) = zernikeCorrScorePolar(A1_nm(:,psel1),A1_nm(:,psel1),Zbasis);
smatrix2(psel2,psel2) = zernikeCorrScorePolar(A2_nm(:,psel2),A2_nm(:,psel2),Zbasis);

sel = repmat(nan,size(psel1)); % allocate unique selection index
for ii=1:length(psel1);
  % psel1(ii) & psel2(ii) represent a putative correspondence pair based
  % upon a feature similarity score and a forwards and backwards mapping.
  
  % find indicies of the other features candidates involved in the pose
  % constrained matching
  fsel1 = find(cmatrix(:,psel2(ii))); % candidate features in I1 for psel2(ii)
  fsel2 = find(cmatrix(psel1(ii),:)); % candidate features in I2 for psel1(ii) 
  
  % remove self-index from the pose constrained feature list
  fsel1(fsel1 == psel1(ii)) = [];
  fsel2(fsel2 == psel2(ii)) = [];
  
  % check for the local "intra-image uniqueness" of each feature involved in the
  % putative match (psel1(ii),psel2(ii)).  e.g. the feature psel1(ii) is
  % defined to be locally "unique" if it doesn't have a high feature
  % similarity score with any other pose constrained *intra* image features.
  if any(smatrix1(psel1(ii),fsel1) > thresh) && ...
     any(smatrix2(psel2(ii),fsel2) > thresh);
    % the features involved in this match were determined to not be
    % locally unique, therefore don't add them to the list of correspondences
    % to keep
  else;
    sel(ii) = ii;
  end;
end;

% remove non-used elements in the pre-allocated vector sel
sel = sel(~isnan(sel));
