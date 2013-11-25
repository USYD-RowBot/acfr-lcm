path = [
-5, 10, 1.9; 
5, 10, 1.9; 
5, -10, 1.9; 
0, 0, 0; 
];
figure; clf; grid on; axis equal; hold all;
plot3d( path', 'c' )
scatter3( path(:,1), path(:,2), path(:,3), 20, 1:size(path,1) )
xlabel( 'x [m]' ); ylabel( 'y [m]' );
view(0,90)
