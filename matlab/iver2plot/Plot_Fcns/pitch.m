function centerY = pitch(y_imgs, iver_t, t_imgs)
%-----------------------------------------------------------------------
%Michelle Howard
%6/27/11
%This function outputs the Y center of the  camera field of view based on
%the pitch angle and height from the bottom of Iver
%Input: pitch( y_imgs, iver_t, t_imgs )
%----------------------------------------------------------------------

  
pitch = interp1(iver_t.unixtime, iver_t.comp.Pitch_Angle, t_imgs);
Y = interp1(iver_t.unixtime, iver_t.alt.DTB_Height, t_imgs); %a matrix of the distance from the bottom
n = 180 - pitch;
F  = (Y./sind(90-n)).*sind(n);
centerY = y_imgs+F;

end