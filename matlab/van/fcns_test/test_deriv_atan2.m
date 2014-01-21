function test_deriv_atan2
% this script just illustrates that the derivative of atan and atan2 are
% the same.

format long;

theta = [0:30:360]*DTOR;

for ii=1:length(theta)
  x = cos(theta(ii));
  y = sin(theta(ii));
  X = [x;y];

  z1 = mytan(X);
  J1 = numerical_jacobian(@mytan,X,z1);

  z2 = mytan2(X);
  J2 = numerical_jacobian(@mytan2,X,z2);

  disp([theta(ii)*RTOD,NaN;z1,z2;J1;J2]);
end

format short;

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function z = mytan(X);
x = X(1);
y = X(2);
z = atan(y/x);


function z = mytan2(X);
x = X(1);
y = X(2);
z = atan2(y,x);
