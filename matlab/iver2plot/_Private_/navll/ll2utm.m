function [X,Y] = ll2utm(lat,longit,zone)

% reference, Snyder, Map Projections--A Working Manual, p. 61 */

%    lat, longit ; the input latitude and longitude, in decimal degrees */
%     zone;        		the input zone, which will automatically control */

%    X,Y;       	the output UTM X and Y, in meters */

   RTOD = 180/pi;
   DTOR = pi/180;
   RADIUS=			6378137.0;
	FLATTENING	=	0.00335281068; % GRS80 or WGS84 */
 	K_NOT		=	0.9996;	      % UTM scale factor */
   lat = lat*DTOR;
   longit = longit*DTOR;
% check for validity of parameters */

%  if(fabs(lat) > 1.56)
%	  return(-1); /* the UTM will blow up at the equator */
%  if((zone < 1)||(zone > 60))
%	  return(-2);  /* zone is only valid between 1 and 60 */
  
% first compute the necessary geodetic parameters and constants*/

  lambda_not = ((-180.0 + zone*6.0) -3.0)/RTOD ;
  e_squared = 2.0 * FLATTENING -FLATTENING* FLATTENING;
  e_fourth = e_squared * e_squared;
  e_sixth = e_fourth * e_squared;
  e_prime_sq = e_squared/(1.0 - e_squared);
  sin_phi = sin(lat);
  tan_phi = tan(lat);
  cos_phi = cos(lat);
  N = RADIUS/sqrt(1.0 - e_squared*sin_phi*sin_phi);
  T = tan_phi*tan_phi;
  C = e_prime_sq*cos_phi*cos_phi;
  M = RADIUS*((1.0 - e_squared*0.25 -0.046875*e_fourth  -0.01953125*e_sixth)*lat- ...
	      (0.375*e_squared + 0.09375*e_fourth + ...
				 0.043945313*e_sixth)*sin(2.0*lat) + ...
	      (0.05859375*e_fourth + 0.043945313*e_sixth)*sin(4.0*lat) - ...
	      (0.011393229 * e_sixth)*sin(6.0*lat));
  A = (longit - lambda_not)*cos_phi;
  A_sq = A*A;
  A_fourth = A_sq*A_sq;
  
%  now go ahead and compute X and Y */
  
  X = K_NOT*N*(A + (1.0 - T + C)*A_sq*A/6.0 + ...
		   (5.0 - 18.0*T + T*T + 72.0*C - ...
		    58.0*e_prime_sq)*A_fourth*A/120.0);
  
% note:  specific to UTM, vice general trasverse mercator.  since the origin 
%	    is at the equator, M0, the M at phi_0, always equals zero, and I
%	    won't compute it                                                */
  
  Y = K_NOT*(M + N*tan_phi*(A_sq/2.0 +  ...
			    (5.0 - T + 9.0*C + 4.0*C*C)*A_fourth/24.0 + ...
			    (61.0 - 58.0*T + T*T + 600.0*C -  ...
			     330.0*e_prime_sq)*A_fourth*A_sq/720.0));
  
%   now correct for false easting and northing */
  
  if( lat < 0)
	  Y = Y +10000000.0;
   end
  X = X + 500000;
