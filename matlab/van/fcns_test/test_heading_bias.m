theta = [0:1:360]*DTOR;

biasN =  3*DTOR;
biasS = -3*DTOR;

theta_new = theta + biasN*abs(cos(theta/2)) + biasS*abs(sin(theta/2));

theta = theta*RTOD;
theta_new = theta_new*RTOD;

figure(1);
plot(theta,theta,'b',theta,theta_new,'r');
grid on;
