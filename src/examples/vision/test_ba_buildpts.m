function [x3d, idx] = buildpts_testsba(dof,h,offset)

% x3d=[x1 x2 x3 ...
%      y1 y2 y3 ...
%      z1 z2 z3 ...     3xN matrix
% branching from buildpts
% to give more 3d variation of the synthetic data
x3d=[];
lengthx=2; lengthy=2; %[m]

dx = 0.1; %5cm 0.01
dy = 0.1; 
dz = 0.05; % 0.025
% h = 0.5;

x=-lengthx:dx:lengthx;
y=-lengthy:dy:lengthy;
if dof == 3
    for ii=1:1:size(x,2);
        for jj=1:1:size(y,2);
            z =  3*(1-x(ii)).^2.*exp(-(x(ii).^2) - (y(jj)+1).^2) ...
                - 10*(x(ii)/5 - x(ii).^3 - y(jj).^5).*exp(-x(ii).^2-y(jj).^2) ...
                - 1/3*exp(-(x(ii)+1).^2 - y(jj).^2);
            %         z = 0.1 + (h-0.1) * rand;
            x3d=[x3d [x(ii) y(jj) offset+h*z;]'];
        end
    end
else
    for ii=1:1:size(x,2);
        for jj=1:1:size(y,2);
            x3d=[x3d [x(ii) y(jj) offset]'];
        end
    end
end

idx=1:1:size(x3d,2);

