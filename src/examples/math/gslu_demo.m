function gslu_demo ()

%=========================
% gsl_util_vector.h
%=========================

x = [0 0 1 0]'

y = x

isel = 3:4;

x_sub = x(isel)

y(1:2) = x_sub

sum_ = sum (y)

dist = sqrt (sum (y-x).^2)

a = [1, 2, 3]';
b = [6, 5, 4]';
c = cross (a, b)

stp = a'* cross (b, c)

vtp = cross (a, cross (b, c))

%=========================
% gsl_util_matrix.h
%=========================
A = [4, 10, 7, 4; 0, 1, 7, 6; 7, 5, 7, 1; 8, 6, 7, 1]
B = [9 1 8 2; 10 3 2 3; 5 8 9 6; 1 3 3 5];
C = reshape (0:15, 4, 4)'


A'

D = C(isel,:)

E = C(:,isel)

C(:,3:4) = E

C(3:4,:) = D


det_ = det(A)

trace_ = trace(A)

Ainv = A^-1

C_2x8 = reshape (C, 2, 8)

C_2x8T = reshape (C', 2, 8)

c = C(:)

Cprime = reshape (c, 4, 4)


t = [1, 2, 3]';
skew = skewsym (t)

%=========================
% gsl_util_blas.h
%=========================


b = A*x

mahal_dist2 = (x-b)'*Ainv*(x-b)


bT = x'*A

scalar = x'*A*y

C1 = A*B
C2 = A*B'
C3 = A'*B
C4 = A'*B'

D1 = A*B*C
D2 = A*B*C'
D3 = A*B'*C
D4 = A'*B*C
D5 = A*B'*C'
D6 = A'*B'*C
D7 = A'*B*C'
D8 = A'*B'*C'

%=========================
% gsl_util_linalg.h
%=========================
[Q,R] = qr (A)

[U,S,V] = svd (A)

[U2,S2,V2] = svd (A(1:3,1:4))