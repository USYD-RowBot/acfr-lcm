function plot_footprints(x,y,z,h,a)
% x,y,z world
% h compass


[world_x,world_y,world_z,Xc,Yc,Zc] = camera_positions(x,y,z,h,a);
% plot camera footprints
plot3(world_x', world_y',world_z');
% plot image centers
hold on;
plot3(Xc,Yc,Zc,'+');
hold off;
axis equal;

function [world_x,world_y,world_z,Xc,Yc,Zc] = camera_positions(x,y,z,h,a)
    
FOV_HORIZONTAL = 42.4*pi/180; 
FOV_VERTICAL = 34.5*pi/180;
x_axis_offset = 1.2;  %camera-doppler x axis offset in meters
bboxh = 2*a*tan(FOV_HORIZONTAL/2);
bboxv = 2*a*tan(FOV_VERTICAL/2);
    
len = length(x);
Z = zeros(len,1);
box_y = [Z,bboxh,bboxh,Z,Z]-repmat(bboxh/2,[1 5]);
box_x = [Z,Z,bboxv,bboxv,Z]-repmat(bboxv/2,[1 5]);
box_z = repmat(z-a,1,5);
    
%convert compass heading to cartesian heading
car_heading = -(h*pi/180 - pi/2);

c = cos(car_heading);
s = sin(car_heading);
    
off_x = c*x_axis_offset;
off_y = s*x_axis_offset;

world_x = repmat(x,1,5) + repmat(c,1,5).*box_x + ...
	  repmat(s,1,5).*box_y + repmat(off_x,1,5);
world_y = repmat(y,1,5) - (-repmat(s,1,5).*box_x + ...
			    repmat(c,1,5).*box_y) + repmat(off_y,1,5);

world_z = box_z;
% camera centers
Xc = (world_x(:,1)+world_x(:,2)+world_x(:,3)+world_x(:,4))/4;
Yc = (world_y(:,1)+world_y(:,2)+world_y(:,3)+world_y(:,4))/4;
Zc = world_z(:,1);
