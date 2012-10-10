x = 37.5*DTOR;

k = [0; 10.^[0:10]'];

%y = sin(x)
%dy/dx = cos(x)

y = sin(x+2*pi*k);
dydx_analytical = cos(x+2*pi*k);
dydx_numerical  = diag(numerical_jacobian(@sin,x+2*pi*k));

fprintf('\n');
fprintf('y = sin(x)  dy/dx = cos(x)  x = %g [deg]\n',x*RTOD);
fprintf('x+k*2pi\t    analytical\t  numerical\n');
fprintf('----------------------------------\n');
for ii=1:length(k)
  fprintf('k=%0.0e\t  dy/dx= %06.4f\t  %06.4f\n', ...
	  k(ii),dydx_analytical(ii),dydx_numerical(ii));
end
fprintf('\n');


x = [0; 10.^[0:10]'];

%y = x^2
%dy/dx = 2x

y = x.^2;
dydx_analytical = 2*x;
dydx_numerical  = diag(numerical_jacobian(inline('x.^2'),x));

fprintf('\n');
fprintf('y = sin(x)  dy/dx = 2x\n')
fprintf('x            analytical\t  numerical\n');
fprintf('----------------------------------\n');
for ii=1:length(k)
  fprintf('%0.0e\t  dy/dx= %g\t  %g\n', ...
	  x(ii),dydx_analytical(ii),dydx_numerical(ii));
end
fprintf('\n');
