function bucket_t = initbucket(u,v,b)
%INITBUCKET initialize regularized random sampling bucket.
%   B = INITBUCKET(u,v,b) regularly divides the feature points (u,v)
%   into a [b x b] grid and returns a bucket structure B.
%
%   Based upon:
%   Zhang, Z., et al.  A Robust Technique for Matching Two Uncalibrated Images
%   Through the Recovery of the Unknown Epipolar Geometry.  Technical
%   Report #2273.  May 1994.
%
%   See also RANDBUCKET.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-26-2003      rme         Created.
%    04-06-2004      rme         Added some more code comments.

N = length(u);

umin = floor(min(u)); umax = ceil(max(u));
vmin = floor(min(v)); vmax = ceil(max(v));

% grid bucket intervals
xbin = linspace(umin,umax,b+1); % b "buckets"
ybin = linspace(vmin,vmax,b+1); % b "buckets"

% init bucket structure
bucket_t.sel = zeros(N,1);
bucket_t.ind = zeros(b^2,2);
bucket_t.bin = zeros(b^2,1);

cb = 1;    % current bucket count
cum = 0;   % cumulative weight count
count = 0; % interest point count
for i=1:b
  % interest points lying within horiz dim of bucket
  selu = find(xbin(i) <= u & u < xbin(i+1));
  for j=1:b
    % interest points lying within vert dim of bucket
    selv = find(ybin(j) <= v(selu) & v(selu) < ybin(j+1));
    % index of interest points falling in bucket
    selb = selu(selv);
    % number of points in bucket
    np = length(selb);
    
    if np > 0
      % update bucket structure      
      bucket_t.sel(count+[1:np]) = selb;
      bucket_t.ind(cb,:) = [1 np]+count;
      bucket_t.bin(cb) = cum+np/N;
      
      cb = cb+1;         % increment current bucket count
      cum = cum+np/N;    % increment cumulative weight count
      count = count+np;  % increment interest point count
    end
  end
end

% only keep used indices
bucket_t.sel = bucket_t.sel(1:count);
bucket_t.ind = bucket_t.ind(1:cb-1,:);
bucket_t.bin = bucket_t.bin(1:cb-1);
