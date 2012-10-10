function points = yaw(centerX, centerY, iver_t,t_imgs) 
%-----------------------------------------------------------------------
%Michelle Howard
%6/27/11
%This function outputs the 4 corners of the field of view based on the
%center of field of view and the yaw angle.
%Input: pitch( centerX, centerY, iver_t, t_imgs)
%----------------------------------------------------------------------

theta = interp1(iver_t.unixtime, iver_t.comp.True_Heading, t_imgs);
theta = -theta - 90;
Y = interp1(iver_t.unixtime, iver_t.alt.DTB_Height, t_imgs);

Xmin = centerX - Y;
Xmax = centerX + Y;
Ymin = centerY - (Y./sind(60)).*sind(40);
Ymax = centerY + (Y./sind(60)).*sind(40);


%Temporarily moves the point to the origin for rotation
tempXmin = Xmin - centerX; 
tempXmax = Xmax - centerX;
tempYmin = Ymin - centerY;
tempYmax = Ymax - centerY;


point1 = [tempXmin, tempYmin];
point2 = [tempXmax, tempYmin];
point3 = [tempXmax, tempYmax];
point4 = [tempXmin, tempYmax];
                    

%Rotates each point around the origin and then moves point back to original
%location
newPoint1 = [(point1(:,1).*cosd(theta) - point1(:,2).*sind(theta))+centerX, (point1(:,1).*sind(theta) + point1(:,2).*cosd(theta))+centerY];
newPoint2 = [(point2(:,1).*cosd(theta) - point2(:,2).*sind(theta))+centerX, (point2(:,1).*sind(theta) + point2(:,2).*cosd(theta))+centerY];
newPoint3 = [(point3(:,1).*cosd(theta) - point3(:,2).*sind(theta))+centerX, (point3(:,1).*sind(theta) + point3(:,2).*cosd(theta))+centerY];
newPoint4 = [(point4(:,1).*cosd(theta) - point4(:,2).*sind(theta))+centerX, (point4(:,1).*sind(theta) + point4(:,2).*cosd(theta))+centerY];

points = [ newPoint1, newPoint2, newPoint3, newPoint4, newPoint1 ];
end