function [bins,occ_idx] = binsort(x,xb,varargin)
%BINSORT  Sort array into regular-spaced cells
%
%  bins = BINSORT(x,xb) sorts the vector x into a cell array whose
%  bins contain all elements of x greater than the corresponding
%  element of xb, but less than the next element of xb. The bin
%  array xb must be regularly spaced.
%
%  bins = BINSORT(x,xb,x1,x2,...,xn) sorts the matrix [x x1 x2 ... xn] 
%  into a cell array as above using x as the sorting criterion, but
%  carrying along the elements of [x1 x2 ... xn].  All additional
%  arguments must have the same number of rows as x, but may have
%  an arbitrary number of columns.  Note that if a vector
%  [1:length(x)]' is input as one of the latter arguments, the bins
%  will retain a record of the original index of the point within x.
%
%  [bins,occ_idx] = BINSORT(...) returns the indecies of occupied
%  bins along with the bins themselves.
%
%  In the interests of speed, BINSORT does not check that input
%  arrays satisfy the conditions above.  Subscripting errors or
%  erratic binning will result.

DX = xb(2)-xb(1);
xx = [x horzcat(varargin{:})];
nbins = length(xb);
bins = cell(nbins,1);
[x,sort_idx] = sort(x);
xx = xx(sort_idx,:);
sorter = [0; diff(floor((x-xb(1))/DX))];
[bin_idx,clearme,diff_value] = find(sorter);
bin_cnt_start = 1+floor((x(1)-xb(1))/DX);
occ_idx = cumsum([bin_cnt_start; diff_value]);
bin_idx = [1; bin_idx; length(x)+1];

for n = 1:length(bin_idx)-1
  bins{occ_idx(n)} = xx(bin_idx(n):bin_idx(n+1)-1,:);
end



