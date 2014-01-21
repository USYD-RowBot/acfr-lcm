% test_dopv2d.m


phi = [0:1:50]*pi/180;

theta = 30*pi/180;
l_T = [0,1,-50]';
x_o = 0;
v_o = 1;
table = [];
table = zeros(length(phi),3);
for ii=1:length(phi)
  b_B = cos(phi(ii));
  a_B = sqrt(1-b_B^2);
  l_B = [a_B,b_B,0]';
  [vs_meas,Vr_meas,Xdot_L,Xdot_R] = dopv2d(l_T,l_B,x_o,v_o,theta,1);
  table(ii,:) = [phi(ii)*180/pi, vs_meas'];
end
figure(2);
plot(table(:,1),(table(:,2)-v_o)/v_o,'rx',table(:,1),table(:,3)/v_o,'bx',...
     table(:,1),(table(:,2)-v_o)/v_o,'r',table(:,1),table(:,3)/v_o,'b');
legend('e_x','e_z');
xlabel('Bottom Inclination');
ylabel('Normalized Error Velocity');
grid on;
fprintf('true vs_x  vs_z\n');
disp([v_o 0]);
fprintf('meas deg      vs_x     vs_z\n');
disp(table);
