function plot_trajsim(X,t,mindx,dh,fignum)
  
if ~exist('fignum','var')
  fignum = figure;
end

% plot results
figure(fignum);
plot(X(2,:),X(1,:));
grid on;
axis equal;
zoom on;
title('XY Local-Level Plot');
ylabel('X_{LL} [m]');
xlabel('Y_{LL} [m]');

fignum = fignum+1;
figure(fignum);
plot(t,X(3,:));
grid on;
zoom on;
title('Z Plot');
xlabel('t [seconds]');

fignum = fignum+1;
figure(fignum);
plot(t,sqrt(X(7,:).^2+X(8,:).^2));
grid on;
zoom on;
title('Speed Plot');
xlabel('t [seconds]');
ylabel('Speed [m/s]');

fignum = fignum+1;
figure(fignum);
plot(t,X(6,:)*RTOD);
grid on;
zoom on;
title('Heading Plot');
xlabel('t [seconds]');
ylabel('Heading [degrees]');

fignum = fignum+1;
figure(fignum);
plot(t,X(4,:)*RTOD,'b',t,X(5,:)*RTOD,'r');
grid on;
zoom on;
title('Roll/Pitch Plot');
xlabel('t [seconds]');
ylabel('Roll/Pitch [degrees]');
legend('Roll','Pitch',0);

fignum = fignum+1;
figure(fignum);
plot(t,dh*RTOD,'b',t,X(6,:)*RTOD,'r');
grid on;
zoom on;
title('Desired Heading Plot');
xlabel('t [seconds]');
ylabel('Heading [degrees]');
legend('dh','actual',0);
