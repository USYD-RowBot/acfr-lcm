function key = sift(I,maxNumKeys,showKeys);
%SIFT Scale Invariant Feature Transform
%   This m-file serves as a wrapper to David Lowe's SIFT feature detector.
%
%   key = sift(I,maxNumKeys,showKeys) returns a keypoint data structure
%   associated with the image I.
%   key.descriptor: [128xN] matrix integers in the range [0, 255]
%   key.vector:     [4xN] matrix of floats
%                   [subpixel_row; subpixel_column; scale; orientation]
%                   orientation is in range [-PI, PI] radians
%
%            I: graylevel image to process
%   maxNumKeys: orders keys in decreasing order of scale and returns up to
%               specified number of keys
%     showKeys: binary flag for displaying keys
%
%   Note: subpixel_row and subpixel_column are defined such that the center
%         of the top left pixel is (0,0), Matlab's default is (1,1).
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-08-2004      rme         Created based upon the sift.m m-file distributed with
%                                David Lowe's SIFT demo code.
%    12-16-2004      rme         Updated help file.
%                                Store keypoint descriptors as uint8.
%    01-07-2005      rme         Use imwrite instead of savepgm.m
%    01-13-2006      rme         Changed eval() to system() call.

if ~exist('showKeys','var') || isempty(showKeys); showKeys = false; end;
if ~exist('maxNumKeys','var') || isempty(maxNumKeys); maxNumKeys = inf; end;
if nargout == 0; showkeys = true; end;

% convert image to a 8-bit PGM temporary image file readable by "keypoints" exectuable
I = im2uint8(I);
fname = tempname;
imwrite(I,[fname,'.pgm'],'pgm');

% get full path name of keypoints executable
persistent keypointsExecutable;
if isempty(keypointsExecutable);
  pathstr = fileparts(which('sift.m'));
  keypointsExecutable = [pathstr,'/lowe_demoV3/keypoints'];
end;

% call keypoints executable
cmd = sprintf('%s < %s.pgm > %s.key',keypointsExecutable,fname,fname);
system(cmd);

% open keypoint file and check its header
fid = fopen([fname,'.key'],'r');
if fid == -1;
  error(['Could no open temporary keypoints file ',fname,'.key']);
end;
[header, count] = fscanf(fid,'%d %d', [1 2]);
key.num  = header(1);
key.size = header(2); % should be 128
if count ~= 2 || key.size ~= 128;
  error(['Invalid keypoint file header in ',fname,'.key']);
end;

% allocate ouput data structure
key.descriptor = repmat(uint8(0),key.size,key.num);
key.vector     = zeros(4,key.num);

% read keypoint file
for ii=1:key.num;
  % [subpixel_row, subpixel_col, scale, orientation]
  [key.vector(:,ii), count] = fscanf(fid, '%f %f %f %f', [1 4]);
  if count ~= 4;
    error('Invalid keypoint file format');
  end;
  
  % descriptor
  [key.descriptor(:,ii), count] = fscanf(fid, '%d', [1 key.size]);
  if count ~= key.size || any(key.descriptor(:,ii) < 0) || any(key.descriptor(:,ii) > 255);
    error('Invalid keypoint file format');
  end;
end;
fclose(fid);

% order keys in descending order by scale and keep only up to maxNumKeys
[scale,ii] = sort(-key.vector(3,:));
if length(ii) > maxNumKeys;
  ii = ii(1:maxNumKeys);
  fprintf('%d keypoints returned.\n',maxNumKeys);
end;
key.num = length(ii);
key.vector = key.vector(:,ii);
key.descriptor = key.descriptor(:,ii);

% delete temporary files
delete([fname,'.*']);

if showKeys;
  drawsift(I,key);
end;
