function [estruc,kreal,cond4] = estim_E_6p(D)
% ESTIM_E_6P  Calculate E based upon 6 point algorithm.
% Similar to Hofmann-Wellenhof Method [1] but using four of the
% nine Demazure constraints
% 
%  [ESTRUCT,K] = ESTIM_E_6P(D) returns a structure ESTRUCT which
%  contains K solutions where K is either 2 , 4, or 6.
%  i.e. ESTRUCT(K).E and ESTRUCT(K).e  ([3x3] and [9x1] respectively)
%  D is a [7x9] measurement matrix with rows of the form:
%  [x2*x1 x2*y1 x2 y2*x1 y2*y1 y2 x1 y1 1]
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-2002         op          Created and written.
%    09/15/2003      rme         ryanized oscar's code


% solve D*e = 0 via singular value decomposition
[U,S,V] = svd(D);

% null space vectors of D
ea = V(:,7);
eb = V(:,8);
ec = V(:,9);

% reshape into [3x3] Essential matrices
Ea = reshape(ea,3,3)';
Eb = reshape(eb,3,3)';
Ec = reshape(ec,3,3)';

% the solution E is a linear combination of the null space vectors and is
% of the form: E = a*Ea + b*Eb + c*Ec
% Since E is only defined up to scale we can rewrite the system as:
%              E = alpha*(Ea-Ec) + beta*Eb + Ec
% To solve for alpha and beta we apply the Demazure constraints
% i.e. (E*E'-trace(E*E')/2)*E = 0 
% which enforce two equal and one zero singular values.
%      
% The Demazure constraints are cubic in alpha and beta

% generate system to solve for weights
[A,cond4] = demazure_constraints_triggs((Ea-Ec),Eb,Ec);

% solve by generating a polynomial in beta and then in alpha
[alpha,beta] = weights_6p(A); % SVD 4 most relevant equations expanded shift

% the above solution for alpha and beta can have up to six roots
num_of_solns = length(alpha);
kreal = 0;
if num_of_solns > 1
  for k = 1:num_of_solns
    if isreal(alpha(k))
      kreal = kreal+1;
      E = alpha(k)*(Ea-Ec) + beta(k)*Eb + Ec;
      
      %[U,S,V] = svd(Eapp);
      %if (S(1,1)/S(2,2) < 1.1) & (S(3,3) < 0.01) 
      %sv = 0.5*(S(1,1)+S(2,2));
      %S = [sv 0 0; 0 sv 0; 0 0 0];
      %E = U*S*V';
      %else
      %	E = ones(3);
      %end
      
      estruc(kreal).E = E;
      estruc(kreal).e = reshape(E',[9 1]);
    end
  end
else % no output can happen due to noise
  estruc = [];
end



%***************************************************************************
function [N,cond4] = demazure_constraints_triggs(Ex,Ey,Ez)
% This function is copied form a Matlab implementation of a linear 6-point
% algorithm developed by Bill Triggs.  The original source file is called
% Emat_from_pts_lin.m
%
% The snippet of code below was taken from Emat_from_pts_lin and builds the
% [9x10] system of equations which enforce the Demazure constraints.
  
Nxx = Ex*Ex'; Nxx = Nxx - trace(Nxx)*eye(3)/2;
Nyy = Ey*Ey'; Nyy = Nyy - trace(Nyy)*eye(3)/2;
Nzz = Ez*Ez'; Nzz = Nzz - trace(Nzz)*eye(3)/2; 
Nxy = Ex*Ey'; Nxy = Nxy+Nxy'-trace(Nxy)*eye(3);
Nxz = Ex*Ez'; Nxz = Nxz+Nxz'-trace(Nxz)*eye(3);
Nyz = Ey*Ez'; Nyz = Nyz+Nyz'-trace(Nyz)*eye(3);
Nxxx = Nxx*Ex;
Nyyy = Nyy*Ey;
Nzzz = Nzz*Ez;
Nxxy = Nxx*Ey+Nxy*Ex;
Nxxz = Nxx*Ez+Nxz*Ex;
Nxyy = Nyy*Ex+Nxy*Ey;
Nxzz = Nzz*Ex+Nxz*Ez;
Nyyz = Nyy*Ez+Nyz*Ey;
Nyzz = Nzz*Ey+Nyz*Ez;
Nxyz = Nxy*Ez+Nyz*Ex+Nxz*Ey;
N = reshape([Nxxx,Nxxy,Nxxz,Nxyy,Nxyz,Nxzz,Nyyy,Nyyz,Nyzz,Nzzz],9,10);
 
[U,S,V] = svd(N);
cond4 = S(4,4)/S(1,1);


%***********************************************************************
function [xroots,yroots] = weights_6p(A);
% this function was orignally written as a stand-alone m-file by oscar and was
% titled weights_6p2.m.  

% Perform Gauss-Seidel elimination on the system
% of Demazure constraints
%-----------------------------------------------  
[U,S,V] = svd(A);
As = V(:,1:4)';
Ar = rref2(As,4);
% last 3 equations x^2*y x^2 x*y^2 x*y x y^3 y^2 y 1
B = Ar(2:4,2:10);
% expand to x^2*y x^2 x*y^2 x*y x y^6 y^5 y^4 y^3 y^2 y 1
C = [B(:,1:5) zeros(3,3) B(:,6:9)];
% eliminate x^2*y term of first row by shifting second row and substracting
C(1,:) = C(1,:) - [C(2,2:12) 0];

% keep rows one and three x*y^2 x*y x y^6 y^5 y^4 y^3 y^2 y 1
D(1,:) = C(1,3:12);
D(2,:) = C(3,3:12);
     
% make first equation a function of x*y^2  0   x y^6 y^5 y^4 y^3 y^2 y 1
% and second equation a function of   0   x*y  x y^6 y^5 y^4 y^3 y^2 y 1
Dr = rref(D);

% eliminate x*y^2 term by shifting and substracting second equation
E(1,:) = Dr(1,:) - [Dr(2,2:10) 0];
E(2,:) = Dr(2,:);

% make first equation a function of x^y 0 y^6 y^5 y^4 y^3 y^2 y 1
% and second equation a function of  0  x y^6 y^5 y^4 y^3 y^2 y 1
Er = rref(E);

% elimante x*y term and x term by shifting and substracting second equation
F(1,:) = Er(1,:) - [Er(2,2:10) 0];
F(2,:) = Er(2,:);

% extract polynomial of order 6 from first row and solve for roots
ypoly = F(1,4:10);
yroots = roots(ypoly);
yroots = yroots(~(imag(yroots))); % pick real roots

% equation 2 determines x as a function of y
xpoly = F(2,5:10);
xroots = -polyval(xpoly,yroots); % evaluate real roots of y to find x



%*************************************************************************
function [A,jb] = rref2(A,m2,tol)
%RREF2   Reduced row echelon form.
[m,n] = size(A);

% Compute the default tolerance if none was provided.
if (nargin < 3), tol = max(m,n)*eps*norm(A,'inf'); end

% Loop over the entire matrix.
i = 1;
j = 1;
jb = [];
while (i <= m2) & (j <= n)
   % Find value and index of largest element in the remainder of column j.
   [p,k] = max(abs(A(i:m,j))); k = k+i-1;
   if (p <= tol)
      % The column is negligible, zero it out.
      A(i:m,j) = zeros(m-i+1,1);
      j = j + 1;
   else
      % Remember column index
      jb = [jb j];
      % Swap i-th and k-th rows.
      A([i k],j:n) = A([k i],j:n);
      % Divide the pivot row by the pivot element.
      A(i,j:n) = A(i,j:n)/A(i,j);
      % Subtract multiples of the pivot row from all the other rows.
      for k = [1:i-1 i+1:m]
         A(k,j:n) = A(k,j:n) - A(k,j)*A(i,j:n);
      end
      i = i + 1;
      j = j + 1;
   end
end
