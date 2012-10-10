function distsq = siftSimilarityScore(keyi,keyj);
%function distsq = siftSimilarityScore(keyi,keyj);  
%
%    INPUTS
%    keyi, keyj: key structures returned by sift.m
%
%    OUTPUTS
%    distsq: [MxN] similarity matrix of squared Euclidean distance
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-18-2004      rme         Created from matchsift.m

% convert uint8 to double for calculations
keyi.descriptor = double(keyi.descriptor);
keyj.descriptor = double(keyj.descriptor);

% the squared Eucldiean distance between two vectors x & y can be written as:
% (x-y)'*(x-y) = x'*x - 2*x'*y + y'*y
%
% compute Euclidean distance between all keypairs
xTx = sum(keyi.descriptor.*keyi.descriptor,1);
yTy = sum(keyj.descriptor.*keyj.descriptor,1);
xTy = keyi.descriptor'*keyj.descriptor;
distsq = repmat(xTx',[1 keyj.num]) - 2*xTy + repmat(yTy,[keyi.num 1]);
