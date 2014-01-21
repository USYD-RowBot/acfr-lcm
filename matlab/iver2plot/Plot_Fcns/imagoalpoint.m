% This function is created to accomodate another button in the plot field
% Who           When        What  
% ------------------------------
%  sbs        May 25,2010   creatd the file to add goal-poits as an option 
%                           in plotting data 
function imagoalpoint()

% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
    errordlg('This plot requires UVC log data','Error');
    return;
end

% timebase hack 
iver_t.elaptime(1) = 0;
iver_t.elaptime(end) = iver_t.ENDTIME-iver_t.STARTTIME;

% draw the goal points on the plot
xref = iver_t.xref;
yref = iver_t.yref;
ref_time = iver_t.elaptime;

[xref_waypoint, yref_waypoint] = collapse(xref, yref); % the co-ordinate of the way points

num_waypoints = length(xref_waypoint);
lengthofdata = size(ref_time,1);  % number of data
way_points_time = zeros(1,num_waypoints) ;

% the following finds out at which index does the way point occurs
way_points_time (1) = ref_time(1);
for j = 2: num_waypoints
    for i = 1:lengthofdata
        if ((xref(i)==xref_waypoint(j-1)) && (xref(i+1)==xref_waypoint(j)))
            way_points_time (j) = ref_time(i+1);
            break
        end
    end
end

% get plot axes
xextents = get(gca,'XLim');
yextents = get(gca,'YLim');
xrange = xextents(2) - xextents(1);
yrange = yextents(2) - yextents(1);

% plotting the waypoints as vertival lines
hold on; 
for j = 1:num_waypoints
    line([way_points_time(j) way_points_time(j)],[yextents(1) yextents(2)],'color','g','LineWidth',2)
    text(way_points_time(j),yextents(2) - 0.1*yrange, sprintf('%d',j) );
end
hold off
