%
% function interpend was made by DY to eliminate frustration using interp1, which
% interpolates from one time base to another. But interp1 ALWAYS seems to give me
% an error message, which turns out to be misleading.
% for interpend, the arguments are the same as interp1, but it fixes any nonmonotonic
% time vectors and deals gracefully with mismatched ends
% function yi = interpend(x,y,xi)
% x is the original time base, y is the variable that goes with that time base
% xi is the new time base
% the output, yi is y interpolated onto xi
%
function yi = interpend(x,y,xi)
ix=find([0;diff(x)] > 0.0);
x0 = x(ix);
y0 = y(ix);
ni = length(xi);
n0 = length(x0);
ind = find( (xi > x0(1)) & (xi < x0(n0)) & ([diff(xi);0] >= 0.0));
nind = length(ind);
yi = zeros(ni,1);

ii = find(diff(x0) <= 0.0);
%fprintf('%d points are not monotonic in X\n',length(ii));
ii = find(diff(xi(ind)) <= 0.0);
%fprintf('%d points are not monotonic in Xi\n',length(ii));
yi(ind) = interp1(x0,y0,xi(ind));

inds = find(xi <= min(x0));
ns = length(inds);
yi(inds) = ones(ns,1)*y0(1);

inde = find(xi >= max(x0));
ne = length(inde);
yi(inde) = ones(ne,1)*y0(n0);





