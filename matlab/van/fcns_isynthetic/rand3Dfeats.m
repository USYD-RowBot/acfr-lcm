function [X,Y,Z] = rand3Dfeats(Nf,xdim,ydim,zstd);
%generates 3D feature points
%input:
%Nf : number of features
%xdim : size of scene in E-W dir [m]
%ydim : size of scene in N-S dir [m]
%zstd : standard deviation of height [m]

X = xdim*(rand(Nf,1)-1/2);
Y = ydim*(rand(Nf,1)-1/2);
Z = zstd*randn(Nf,1);
