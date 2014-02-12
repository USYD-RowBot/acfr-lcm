% test_singularity.m

if false
t = rand(3,1); P = rand(3); P = (P+P')/2;
m = norm(t);

w = t/m;

Jt = numerical_jacobian(@unitize,t,w);

W1 = Jt*P*Jt';
[U,D,V] = svd(W1); D(3,3) = 0;
W1 = U*D*V';

[U,D,V] = svd(Jt); D(1,1) = (D(1,1)+D(2,2))/2; D(2,2) = D(1,1); D(3,3) = 0;
JJt = U*D*V';
W2 = JJt*P*JJt';

norm(W1-W2)
W1,W2
end


syms x y z real;
t = [x y z]';
t_hat = unitize(t);

Jt_anal = simplify(jacobian(t_hat,t));


t = rand(3,1);

Jt_num = numerical_jacobian(@unitize,t)

Jt_sub = subs(Jt_anal,{x,y,z},{t(1),t(2),t(3)})
