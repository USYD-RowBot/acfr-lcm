function sel = randbucket(b_t,q)
%RANDBUCKET regularized random sampling.
%   SEL = RANDBUCKET(B,q) returns q random indices from a bucket
%   structure B.
%
%   ALGORITHM OVERVIEW
%   The region is evenly divided into [b x b] buckets.  To each bucket is
%   attached a set of points, and indirectly a set of matches, which fall in
%   it.  The buckets having no matches are excluded.  To generate a subsample
%   of q points, we first randomly select q mutually different buckets, and
%   then randomly choose one match in each selected bucket.  If we have in
%   total b buckets, we divide [0 1] into b intervals such that the width of
%   the ith interval is equal to ni/N, where ni is the number of matches
%   attached to the ith bucket and N is the total number of matches.  During
%   the bucket selection procedure, a uniform random number is generated in
%   the interval [0 1], if this number falls in the ith interval then the ith
%   bucket is selected.
%
%   Based upon:
%   Zhang, Z., et al.  A Robust Technique for Matching Two Uncalibrated Images
%   Through the Recovery of the Unknown Epipolar Geometry.  Technical
%   Report #2273.  May 1994.
%
%   See also INITBUCKET.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-26-2003      rme         Created.

if q > size(b_t.bin,1)
  error('Must have more buckets than samples.');
end

bin = [0; b_t.bin];

sel = [];
bsel = [];
nb = 0; % number of uniquely selected buckets
while nb < q
  for i=1:(q-nb)
    % random uniform number in range [0 1]
    samp = rand; 
    % bucket associated with random number
    bcur = find(bin(1:end-1) < samp & samp < bin(2:end));              
    bsel = [bsel; bcur];
    
    % randomly select an interest point from the bucket
    m = b_t.ind(bcur,1);
    n = b_t.ind(bcur,2);
    Npts = n-m+1;
    if Npts > 1
      tsel = randsample(Npts,1);
      sel = [sel; b_t.sel(m+tsel-1)];
    else
      sel = [sel; b_t.sel(m)];
    end
  end % for i=1:(q-nb)

  % keep only unique bucket entries
  [bsel,x,y] = unique(bsel);
  sel = sel(x);
  nb = length(bsel);
end
