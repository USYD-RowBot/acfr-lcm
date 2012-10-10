function [x,y] = pinhole_project(P,X,Y,Z)
%function [x,y] = pinhole_project(P,X,Y,Z)
%or       [x,y] = pinhole_project(P,X);
%projects 3D points X,Y,Z using camera projection matrix P
if nargin == 4;
  I = P*[X, Y, Z, ones(length(X),1)]';
elseif nargin == 2;
  I = P*[X; ones(1,size(X,2))];
end;
x = (I(1,:)./I(3,:))';
y = (I(2,:)./I(3,:))';

