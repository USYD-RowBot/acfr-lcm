function drawsift(I,key,inflate);
%function drawsift(I,key,inflate (opt));
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-08-2004      rme         Created from the sift.m m-file distributed with
%                                David Lowe's SIFT demo code.
%    02-19-2005      rme         Added optional inflate arg for visualization purposes.

if ~exist('inflate','var') || isempty(inflate); inflate=1; end;

% Draw image with keypoints
[m,n] = size(I(:,:,1));
imagesc([0 n-1],[0 m-1],I); % make top left pixel (0,0)
colormap gray; axis image off;

% Draw an unitary horizontal arrow, transformed according to
% keypoint parameters.
TransformLine([m n], key.vector, 0.0,   0.0, 1.0, 0.0, inflate);
TransformLine([m n], key.vector, 0.85,  0.1, 1.0, 0.0, inflate);
TransformLine([m n], key.vector, 0.85, -0.1, 1.0, 0.0, inflate);

%=====================================================================
function TransformLine(im, keypoints, x1, y1, x2, y2, inflate);
% Draw the given line in the image, but first translate, rotate, and
% scale according to the keypoint parameters.
% Function adapted from 'lowe_keys\source\main.c'
%
% Parameters:
%   Arrays:
%    im = [rows columns] of image
%    keypoint = [subpixel_row subpixel_column scale orientation]
%
%   Scalars:
%    x1, y1; begining of vector
%    x2, y2; ending of vector

% The scaling of the unit length arrow is set to half the width
% of the region used to compute the keypoint descriptor.
len = inflate * keypoints(3,:);

% Rotate the keypoints by 'ori' = keypoint(4)
s = sin(keypoints(4,:));
c = cos(keypoints(4,:));

% Apply transform
r1 = keypoints(1,:) - len .* (c * y1 + s * x1);
c1 = keypoints(2,:) + len .* (- s * y1 + c * x1);
r2 = keypoints(1,:) - len .* (c * y2 + s * x2);
c2 = keypoints(2,:) + len .* (- s * y2 + c * x2);

% Discard lines that have any portion outside of image.
line([c1; c2], [r1; r2], 'Color', 'c','linewidth',1.25);
