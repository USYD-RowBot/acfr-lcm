function [X,Y,Z] = rand3Dfeats_planar(Nf,xdim,ydim,zstd);
%generates 3D feature points
%input:
%Nf : number of features
%xdim : size of scene in E-W dir [m]
%ydim : size of scene in N-S dir [m]
%zstd : standard deviation of height [m]

X = xdim*(rand(Nf,1)-1/2);
Y = ydim*(rand(Nf,1)-1/2);
plane_coef= zstd*randn(3,1);
Z = plane_coef(1)*X+plane_coef(2)*Y+plane_coef(3);
