function e = epipole_from_F(F)
%function e = epipole_from_F(F)

[U,S,V] = svd(F);
e = V(:,3);
