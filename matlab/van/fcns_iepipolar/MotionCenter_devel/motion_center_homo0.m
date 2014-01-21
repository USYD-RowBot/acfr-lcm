function [uc,vc,r] = motion_center(u,v,u_prime,v_prime)

% compose homogenous point vectors
x = homogenize(u,v);
x_prime = homogenize(u_prime,v_prime);

% compute homogenous line for each point pair
ell = cross(x,x_prime);
ell = ell./repmat(ell(3,:),[3 1]);

N = length(u);
row = 1;
A = zeros(10000,N);
for ii=1:N-2
  for jj=ii+1:N-1
    for kk=jj+1:N
      A(row,[ii,jj,kk]) = measurement_eqn(ell(:,ii),ell(:,jj),ell(:,kk));
      row = row+1;
    end
  end
end
A = A(1:row-1,:);

[U,S,V] = svd(A);

% compute perpendiculate
ell_perp = ell;
ell_perp([2,1],:) = ell([1,2],:);
ell_perp(1,:) = -ell_perp(1,:);
ell_perp(3,:) = V(:,end)';

[uc,vc] = dehomogenize(cross(ell_perp(:,1),ell_perp(:,2)));
r = [];

function Arow = measurement_eqn(ell1,ell2,ell3)
  
c1 = -ell1(2); d1 = ell2(1);
c2 = -ell2(2); d2 = ell2(1);
c3 = -ell3(2); d3 = ell3(1);

Arow = [(c2*d3-d2*c3), (-c1*d3+d1*c3), (c1*d2-d1*c2)];
