% figure; 
% clf;
grid on; axis equal; 
% hold all;
plot3d( path', 'c' )
hold on
scatter3( path(:,1), path(:,2), path(:,3), 20, 1:size(path,1) )
xlabel( 'x [m]' ); ylabel( 'y [m]' );
view(0,90)
