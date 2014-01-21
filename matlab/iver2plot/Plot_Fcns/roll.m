function centerX = roll(x_imgs, iver_t, t_imgs)
%-----------------------------------------------------------------------
%Michelle Howard
%6/30/11
%This function outputs the X center of the  camera field of view based on
%the roll angle and height from the bottom of Iver
%Input: roll( x_imgs, iver_t, t_imgs)
%----------------------------------------------------------------------

roll = interp1(iver_t.unixtime, iver_t.comp.Roll_Angle, t_imgs);    % Matrix of the Roll Angle of Iver
r = 90 + roll;                                                      % Adjusted (0 degrees to vertical)
Y = interp1(iver_t.unixtime, iver_t.alt.DTB_Height, t_imgs);        % Matrix of the Distance from the Bottom
D = (Y./sind(r)).*sind(90-r);
centerX = x_imgs + D;


end