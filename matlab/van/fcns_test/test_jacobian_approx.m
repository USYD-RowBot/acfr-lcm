function test_jacobian_approx
uvw = rand(3,1)*10
rph = rand(3,1)*90
rph = rph*DTOR;

J1 = numerical_jacobian(@uvw_world,[uvw;rph]);

J2 = zeros(3,6);
J2(1:3,1:3) = rotxyz(rph);
J2(:,4) = rotz(rph(3))'*roty(rph(2))'*deriv_rotx(rph(1))'*uvw; % deriv w.r.t. r
J2(:,5) = rotz(rph(3))'*deriv_roty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. p
J2(:,6) = deriv_rotz(rph(3))'*roty(rph(2))'*rotx(rph(1))'*uvw; % deriv w.r.t. h

e = J1 - J2;
if any(e(:)>eps)
  [J1;J2;e]
end

function uvw_w = uvw_world(x)

uvw = x(1:3);
rph = x(4:6);
uvw_w = rotxyz(rph)*uvw;
