
int utm_geo(double X, double Y, int zone, double *lat, double *longit)

% reference, Snyder, Map Projections--A Working Manual, p. 61 */


DEGREES_TO_RADIANS=	0.01745329252;
FALSE_EASTING=		500000.0;
FALSE_NORTHING=		10000000.0;

%   first, subtract the false easting */

  X = X - FALSE_EASTING;

%  compute the necessary geodetic parameters and constants*/

  e_squared = 2.0 * FLATTENING -FLATTENING* FLATTENING;
  e_fourth = e_squared * e_squared;
  e_sixth = e_fourth * e_squared;
  oneminuse = sqrt(1.0-e_squared);

    

%  compute the footpoint latitude */

  M = Y/K_NOT;
  mu = M/(RADIUS*(1.0 - 0.25*e_squared - 
                  0.046875*e_fourth - 0.01953125*e_sixth));
  e1 = (1.0 - oneminuse)/(1.0 + oneminuse);
  e1sq = e1*e1;
  footpoint = mu + (1.5*e1 - 0.84375*e1sq*e1)*sin(2.0*mu) +
              (1.3125*e1sq - 1.71875*e1sq*e1sq)*sin(4.0*mu) +
              (1.57291666667 *e1sq*e1)*sin(6.0*mu) +
              (2.142578125*e1sq*e1sq)*sin(8.0*mu);


% compute the other necessary terms */

  e_prime_sq = e_squared/(1.0 - e_squared);
  sin_phi = sin(footpoint);
  tan_phi = tan(footpoint);
  cos_phi = cos(footpoint);
  N = RADIUS/sqrt(1.0 - e_squared*sin_phi*sin_phi);
  T = tan_phi*tan_phi;
  Tsquared = T*T;
  C = e_prime_sq*cos_phi*cos_phi;
  Csquared = C*C;
  denom = sqrt(1.0-e_squared*sin_phi*sin_phi);
  R = RADIUS*oneminuse*oneminuse/(denom*denom*denom);
  D = X/(N*K_NOT);
  Dsquared = D*D;
  Dfourth = Dsquared*Dsquared;

  lambda_not = ((-180.0 + (double)zone*6.0) -3.0) * DEGREES_TO_RADIANS;


% now, use the footpoint to compute the real latitude and longitude */

  *lat = footpoint - (N*tan_phi/R)*(0.5*Dsquared - (5.0 + 3.0*T + 10.0*C - 
                                   4.0*Csquared - 9.0*e_prime_sq)*Dfourth/24.0 +
                                   (61.0 + 90.0*T + 298.0*C + 45.0*Tsquared -
                                    252.0*e_prime_sq -
                                    3.0*Csquared)*Dfourth*Dsquared/720.0);
  *longit = lambda_not + (D - (1.0 + 2.0*T + C)*Dsquared*D/6.0 +
                         (5.0 - 2.0*C + 28.0*T - 3.0*Csquared + 8.0*e_prime_sq +
                          24.0*Tsquared)*Dfourth*D/120.0)/cos_phi;




