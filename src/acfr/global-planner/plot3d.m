function plot3d( odo, varargin )

zeroStart = false;
plotArg = 'r-';
% interpret the input args
% for i = 1:length( varargin )
% 	% bringToZero arg
% 	if islogical( varargin{i} )
% 		zeroStart = varargin{i};
% 	end
% 	
% 	% plot arg
% 	if ischar( varargin{i} )
% 		plotArg = varargin{i};
% 	end
% end

% Transform so the plot starts from (0,0)
if zeroStart == true
	odo = bringToZero( odo );
end

% finally plot
plot3( odo(1,:), odo(2,:), odo(3,:), varargin{:} )
view( -30, 30 )
%axis equal
grid on
xlabel( 'x' )
ylabel( 'y' )
zlabel( 'z' )
