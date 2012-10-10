function [uc,vc,r] = motion_center(u1,v1,u2,v2)

% compose vectors
x1 = [u1'; v1'];
x2 = [u2'; v2'];

% observation matrix
A = (x2 - x1)';

% measurement vector
m = length(u1);
b = zeros(m,1);
for ii=1:m
  b(ii) = 0.5*(x2(:,ii) - x1(:,ii))'*(x2(:,ii)+x1(:,ii));
end
%b = 0.5*(x2 - x1)'*(x2 + x1);

% center of motion
y = pinv(A)*b;
uc = y(1);
vc = y(2);

% residual
r = A*y-b;
