A = imread('cameraman.tif');

figure(1);
imshow(A);

% if T is empty, tformarray operates as a direct resampling function
T = [];

% tdims maps row,column space to x-y space
TDIMS_A = [2 1];
TDIMS_B = [2 1];

% resampler options
interp = 'linear';   %{'nearest','linear','cubic'}
border = 'fill';     %{'circular','replicate','symmetric','fill','bound'}
% make resampler
R = makeresampler(interp,border); 

% fill values
F = 0; %black


% CHANGE
switch 3
 %-------------------------------------------------------------------------- 
 case 1 % TWICE AS MANY COLUMNS
  % TMAP_B should be empty if TSIZE_B is not empty
  TMAP_B = [];
  % sample image onto an output space with TWICE as many columns as orig
  TSIZE_B = [512 256];
  B = tformarray(A,T,R,TDIMS_A,TDIMS_B,TSIZE_B,TMAP_B,F);
 %--------------------------------------------------------------------------
 case 2 % HALF AS MANY COLUMNS
  % TMAP_B should be empty if TSIZE_B is not empty
  TMAP_B = [];
  % sample image onto an output space with HALF as many columns as orig
  TSIZE_B = [128 256];
  B = tformarray(A,T,R,TDIMS_A,TDIMS_B,TSIZE_B,TMAP_B,F);
 %--------------------------------------------------------------------------
 case 3 % ARBITRARY LOCATION BOTTOM LEFT CORNER
  TDIMS_B = [1 2];
  % TSIZE_B should be empty if TMAP_B is not empty
  TSIZE_B = [];
  % sample image onto an arbitrary output space
  [x,y] = meshgrid(-64:64,192:320);
  TMAP_B = cat(3,x,y);
  B = tformarray(A,T,R,TDIMS_A,TDIMS_B,TSIZE_B,TMAP_B,F);
 
 
 
 %-------------------------------------------------------------------------- 
end % switch


figure(2);
imshow(B);
