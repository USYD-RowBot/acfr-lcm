function test_triangulate ()

NPOINTS = 10;    
    
K = [1619, 0,    632;
     0,    1617, 512;
     0,    0,    1];

Xo = rand(3, NPOINTS);

[Rt, uu_noise, vv_noise, uu_true, vv_true] = rand_views(2, 5, 1, 1, Xo(1,:)', Xo(2,:)', Xo(3,:)', K);

R = Rt(1:3,1:3);
t = Rt(:,4);

u1 = uu_true(:,1); v1 = vv_true(:,1); uv1 = [u1, v1]';
u2 = uu_true(:,2); v2 = vv_true(:,2); uv2 = [u2, v2]';


[X1,alpha,beta,gamma] = triangulate (R, t, u1, v1, u2, v2, K)

save uv1.txt uv1 -ASCII -DOUBLE
save uv2.txt uv2 -ASCII -DOUBLE
save R.txt R -ASCII -DOUBLE
save t.txt t -ASCII -DOUBLE
save K.txt K -ASCII -DOUBLE
save X1.txt X1 -ASCII -DOUBLE
save alpha.txt alpha -ASCII -DOUBLE
save beta.txt beta -ASCII -DOUBLE
save gamma.txt gamma -ASCII -DOUBLE

