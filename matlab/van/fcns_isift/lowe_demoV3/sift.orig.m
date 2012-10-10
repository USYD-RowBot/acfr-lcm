%[keyDescriptor, keyVector] = sift(imageString)
%
% Finds SIFT features (Lowe, 2004) from inside Matlab
% Made by D. Alvaro and J.J. Guerrero, Universidad de Zaragoza
%   November 2003
%
%@parameter: imageString: image name, the image format doesn't matter
%                         because internally it is converted into PGM.
%
%@return: keyDescriptor: num-by-128 matrix; with 128 integers in range [0, 255]
%                         by a keypoint (each row is a keypoint)
%             keyVector: num-by-4 matrix; each row are 4 floats
%                         [subpixel_row subpixel_column scale orientation]
%                         orientation is in range [-PI, PI] radians
%
% Example: [kd,kv] = sift('book.pgm');


function [keyDescriptor, keyVector] = sift(imageString)

%load image
image = imread(imageString);

[m, n] = size(image); % [rows, columns]

% Convert into PGM imagefile, readable by "keypoints" executable
f = fopen('tmp.pgm', 'w');
if f == -1
    error('Could not create file tmp.pgm.');
end
fprintf(f, 'P5\n%d\n%d\n255\n', n, m);
fwrite(f, image', 'uint8');
fclose(f);

% Call keypoints executable
command = sprintf('!./keypoints <tmp.pgm >tmp.key');
fprintf('Waiting for keypoints ...\n');
eval(command);

%open tmp.key and check its header
g = fopen('tmp.key', 'r');
if g == -1
    error('Could not open file tmp.key.');
end
[header, count] = fscanf(g, '%d %d', [1 2]); %len = 128
if count ~= 2
    error('Invalid keypoint file beginning.');
end
num = header(1);
fprintf('Found %d keypoints.\n', num);
len = header(2);
if len ~= 128
    error('Keypoint descriptor length invalid (should be 128).');
end

%creates two output matrices
keyVector = [];
keyDescriptor = [];

%parse tmp.key
for i = 0: 1: num - 1
    [vector, count] = fscanf(g, '%f %f %f %f', [1 4]); %row col scale ori
    if count ~= 4
        error('Invalid keypoint file format');
    end
    keyVector = [keyVector; vector]; % adds a new row
    
    [descriptor, count] = fscanf(g, '%d', [1 len]);
    descrLessZero = (descriptor(1, :) < 0) * ones(len, 1);
    descrGreater255 = (descriptor(1, :) > 255) * ones(len, 1);
    if (count ~= 128) || descrLessZero || descrGreater255
        error('Invalid keypoint file value.');
    end
    keyDescriptor = [keyDescriptor; descriptor];
end
fclose(g);

% Draw image with keypoints
imshow(imageString);
hold on;
for i = 1: 1: num
    % Draw an unitary horizontal arrow, transformed according to
    % keypoint parameters.
    TransformLine([m n], keyVector(i,:), 0.0, 0.0, 1.0, 0.0);
    TransformLine([m n], keyVector(i,:), 0.85, 0.1, 1.0, 0.0);
    TransformLine([m n], keyVector(i,:), 0.85, -0.1, 1.0, 0.0);
end
hold off;



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
function TransformLine(im, keypoint, x1, y1, x2, y2)

% The scaling of the unit length arrow is set to half the width
% of the region used to compute the keypoint descriptor.
len = 6 * keypoint(3);

% Rotate the keypoints by 'ori' = keypoint(4)
s = sin(keypoint(4));
c = cos(keypoint(4));

% Apply transform
r1 = keypoint(1) - len * (c * y1 + s * x1);
c1 = keypoint(2) + len * (- s * y1 + c * x1);
r2 = keypoint(1) - len * (c * y2 + s * x2);
c2 = keypoint(2) + len * (- s * y2 + c * x2);

% Discard lines that have any portion outside of image.
if r1>=0 && r1<im(1) && c1>=0 && c1<im(2) && r2>=0 && r2<im(1) && c2>=0 && c2<im(2)
    line([c1 c2], [r1 r2], 'Color', 'c');
end
