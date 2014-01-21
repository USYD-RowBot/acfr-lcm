function FF = resize(F,nr,nc,padval)
%RESIZE crops or zero pads an image to the appropriate size.
%   B = RESIZE(A,NR,NC) crops or zero pads image A as necessary to have
%   the output image B have width and height of NC and NR respectively.
%
%   B = RESIZE(A,NR,NC,PADVAL) allows the user to specify the pad value.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-25-2003      rme         Created and written.
  
r = size(F,1);
c = size(F,2);

if ~exist('padval','var')
  padval = 256; % white
end
if r < nr && c < nc
  % pad
  for ii=1:size(F,3) % for each channel
    FF(:,:,ii) = padarray(F(:,:,ii),[nr-r,nc-c],padval,'post');
  end
elseif r >= nr && c >= nc
  % crop
  for ii=1:size(F,3) % for each channel
    FF(:,:,ii) = F(1:nr,1:nc,ii);
  end
elseif r < nr && c >= nc
  % crop c pad r
  for ii=1:size(F,3) % for each channel
    FF(:,:,ii) = padarray(F(:,1:nc,ii),[nr-r, 0],padval,'post');
  end
elseif r >= nr && c < nc
  % crop r pad c
  for ii=1:size(F,3) % for each channel
    FF(:,:,ii) = padarray(F(1:nr,:,ii),[0, nc-c],padval,'post');
  end
end
  
