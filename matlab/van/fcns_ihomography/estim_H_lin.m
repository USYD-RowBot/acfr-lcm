function H = estim_H_lin(x1,y1,x2,y2)

n_points = length(x1);
A = zeros(3*n_points,9);

U1 = [x1';y1';ones(1,n_points)];

for k = 1:n_points
  A(3*(k-1)+1:3*k,:) = [ 0 0 0          -U1(:,k)'          y2(k)*U1(:,k)';
		         U1(:,k)'         0 0 0           -x2(k)*U1(:,k)';
		        -y2(k)*U1(:,k)'  x2(k)*U1(:,k)'   0 0 0];
end

[U,S,V]  = svd(A);
h = V(:,9);
H = reshape(h,3,3)';
