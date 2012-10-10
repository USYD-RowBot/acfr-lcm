close all;

%% generating points 
% load xyz from existing data or generate
dir = '~/perls/opt/examples/libmath/';
% xyz = load([dir,'plane_demo_xyz_fitplane.txt']);
xyz = load(['./plane_demo_xyz_synthetic.txt']);
% h=0.1; offset = 2; [xyz idx] = buildpts_testsba(2,h,offset);

n = size(xyz,2); x = xyz(1,:); y = xyz(2,:); z = xyz(3,:);

% plot
figure(1); hold on;
for i=1:n, plot3(x(i),y(i),z(i),'.'); end


%% plot plane fitted
%  coeff from matlab code or copied from 

[coeff, min_in] = plane_estim_RANSAC(xyz)
% coeff = [0.0478, -0.0161, -0.8858, 1.0000];

a=coeff(1); b=coeff(2); c=coeff(3); d=coeff(4);
[xp, yp] = meshgrid(-2:0.1:2);
zp = (-a.*xp-b.*yp-d)./c;

for i=1:size(xp,1)
    for j=1:size(xp,1)
        plot3(xp(i,j),yp(i,j),zp(i,j),'ro');
    end
end

%% test code to test projected point
ray=[[0.1,0.2,0.5]' [0.2 0.1 0.5]' [0.1 0.1 0.5]' [0.3 0.1 0.3]'];
line ([0,ray(1,1)], [0,ray(2,1)], [0,ray(3,1)],'LineWidth',3,'Color','g')

t = -d./(ray'*coeff(1:3))

ray2=ray*diag(t);
line ([0,ray2(1,1)], [0,ray2(2,1)], [0,ray2(3,1)],'LineWidth',2,'Color','r')


hold off;
grid on; xlabel('x'); ylabel('y'); zlabel('z'); 
