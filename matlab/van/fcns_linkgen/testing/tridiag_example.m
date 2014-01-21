clear all; clc;

N = 3;

% create symbolic tridiagonal matrix
disp('suppose we have a tridiagonal matrix');
a  = symbmatmake('a',N,1,'real');
b  = symbmatmake('b',N,1,'real'); b = b(2:end);
To = diag(a) - 2*diag(b,1);
To = (To+To')/2

% because of special structure of To, we can consider a UL decomposition
% of the form T = U*D^-1*U'
disp('because of the special structure of To, we can consider a');
disp('decomposition of the form To = U*D^-1*U''');
d = symbmatmake('d',N,1,'real');
U = diag(d) - diag(b,1)
D = diag(d)

disp('where by inspection we see that T = U*D^_1*U'' yields');
disp('d(n) = a(n) and d(i) = a(i)-b(i+1)^2/d(i+1)');
T = U*D^-1*U'


disp('T^-1 is then equal to T^-1 = (U^-1)''*D*U^-1');
Uinv = U^-1
Tinv = Uinv'*D*Uinv

disp('and T^-1*T = I');
I = simple(Tinv*T); I
