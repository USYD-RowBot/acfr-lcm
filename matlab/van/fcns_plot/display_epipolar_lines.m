function display_epipolar_lines(I1,I2,F)
%DISPLAY_EPIPOLAR_LINES  User interactive plotting of epipolar lines.
%   DISPLAY_EPIPOLAR_LINES(I1,I2,F) I1 and I2 are two images of the
%   same size.  F is the fundamental matrix which relates the image
%   pair, i.e.  x2 * F * x1 = 0;
%   
%   I1 is displayed in figure 1 and I2 is displayed in figure 2.
%   When a user left clicks on a point in figure 1, the corresponding
%   epipolar line is drawn in figure 2.  To switch roles, click
%   outside the image boundary in I1.  The user can now click on
%   points in I2 and the corresponding epipolar line will be drawn
%   in I1.  To end the user session, simple right click on the
%   current active image.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-06-2002      rme         Created and written.
%    12-13-2002      rme         Center of upper left pixel is defined
%                                to be (0,0) not (0.5,0.5)

nr = size(I1,1);
nc = size(I1,2);

figure(1); clf;
h = imshow(I1,'notruesize');
set(h,'xdata',[1:nc]-1,'ydata',[1:nr]-1);
title('I1');
hold on;
figure(2); clf;
h = imshow(I2,'notruesize');
set(h,'xdata',[1:nc]-1,'ydata',[1:nr]-1);
title('I2');
hold on;

figure(1);
done = 0;
while ~done
  [x,y,button] = ginput(1);
  if button ~= 1
    % users did something other than left click
    % indicating they wish to terminate session
    done = 1;
    continue;
  end
  
  if gcf == 1
    if x < 0 || x > nc || y < 0 || y > nr
      % user clicked outside of image area indicating they
      % wish to switch focus to other window
      figure(2);
      continue;
    end
    x1  = [x y 1]';
    epl2 = F*x1; % epipolar line in image 2 corresponding to x1
    figure(2);
    % plot epipolar line in image 2
    [xx,yy] = line_bounds(0,nc-1,0,nr-1,epl2);
    line(xx,yy,'Color','g');
    % return focus to image 1 and plot point in image 1
    figure(1);
    plot(x,y,'g+');
    
  elseif gcf == 2
    if x < 0 || x > nc || y < 0 || y > nr
      % user clicked outside of image area indicating they
      % wish to switch focus to other window
      figure(1);
      continue;
    end    
    x2  = [x y 1]';
    epl1 = F'*x2; % epipolar line in image 1 corresponding to x2
    figure(1); 
    % plot epipolar line in image 1
    [xx,yy] = line_bounds(0,nc-1,0,nr-1,epl1);    
    line(xx,yy,'Color','y');    
    % return focus to image 2 and plot point in image 2
    figure(2);
    plot(x,y,'y+');
  end
end

figure(1); hold off;
figure(2); hold off;

