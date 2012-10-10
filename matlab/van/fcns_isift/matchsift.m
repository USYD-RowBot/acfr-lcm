function [seli,selj] = matchsift(keyi,keyj,thresh,Ii,Ij);
%function [seli,selj] = matchsift(keyi,keyj,thresh,Ii,Ij);
%
%    INPUTS
%    keyi, keyj: key structures returned by sift.m
%        thresh: distance to next closest match, range [0,1], 0.6 default
%        Ii, Ij: (optional), if given also displays matches
%
%    OUTPUTS
%    seli, selj: selection index of corresponding features
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-09-2004      rme         Created from match.c distributed with
%                                David Lowe's SIFT demo code.
%    12-16-2004      rme         Added conversion of descriptor from
%                                uint8 to double & updated help file
%    12-18-2004      rme         Moved similarity code to siftSimilarityScore.m
 
if ~exist('thresh','var') || isempty(thresh); thresh = 0.6; end;
if ~exist('Ii','var') || ~exist('Ij','var'); 
  showmatch = false;
else;
  showmatch = true;
end;

% compute Euclidean distance between all keypairs
distsq = siftSimilarityScore(keyi,keyj);

% find best match of keyi -> keyj
[seli,selj] = bestmatch(distsq,thresh);

if showmatch;
  fprintf('%d matches found.\n', length(seli));
  [ri,ci] = size(Ii);
  [rj,cj] = size(Ij);
  Iij = Ii;
  Iij(ri+1:ri+rj,1:cj) = Ij;
  imshow([1,max(ci,cj)]-1,[1,ri+rj]-1,Iij);
  title('Ii'); xlabel('Ij');
  line([keyi.vector(2,seli); keyj.vector(2,selj)], ...
       [keyi.vector(1,seli); ri+keyj.vector(1,selj)], 'color','c');
end;

%=========================================================================
function [seli,selj] = bestmatch(distsq,thresh)
% find best match across all columns
[dmin1,ind1] = min(distsq,[],2);

% find next closest match
[m,n] = size(distsq);
lind = sub2ind([m,n],[1:m]',ind1);
distsq(lind) = inf;
dmin2 = min(distsq,[],2);

% only accept match if distance is sufficiently far away from next best match
seli = find(dmin1 < thresh^2*dmin2);
selj = ind1(seli);
