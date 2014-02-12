function [nodes_t, meas_t] = isam2mat (filename, plot_on, save_on)
% function isam2mat (filename, opts)
%
% ex) [nodes_t, meas_t] = isam2mat ('./graph.isam');
%
% nodes_t.n = number of nodes
%        .mu = 6xn matrix
%        .sigma = 36xn matrix
%        .utime = utime
%        .imgfile = path/utime.tif
%
% meas_t.odo_t   = odometry mat structure
% meas_t.cam_t   = camera mat structure
% meas_t.sonar_t = sonar mat structure
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2011.09.23      ak          Moved from matlab branch / created

if nargin < 2
    save_on = 0;
    plot_on = 0;
end

% node parser: n,mu,sigma,utime,imgfilename
nodes_t = isam_pose_parser(filename);

% visually check the nodes loaded
if (plot_on)
    figure; plot3(nodes_t.mu(1,:),nodes_t.mu(2,:),nodes_t.mu(3,:),'.');
    grid on; axis equal;
end

% measurement parser
[odo_t, cam_t, sonar_t, ...
 odo_time, cam_time, sonar_time,...
 odo_idx, cam_idx, sonar_idx] = isam_meas_parser(filename, nodes_t.n);

meas_t.odo_t = odo_t;
meas_t.odo_time = odo_time;
meas_t.odo_idx = odo_idx;
meas_t.cam_t = cam_t;
meas_t.cam_time = cam_time;
meas_t.cam_idx = cam_idx;
meas_t.sonar_t = sonar_t;
meas_t.sonar_time = sonar_time;
meas_t.sonar_idx = sonar_idx;

if save_on
    save nodes_t nodes_t
    save meas_t meas_t 
end
% 
% cross_link = 0;
% for ii=1:length(meas_t.cam_t)
%     cam_t = meas_t.cam_t{ii};
%     dt=abs(cam_t.utime_i-cam_t.utime_j);
%     if dt > 1E6
%         cross_link = cross_link + 1;
%     end
% end
% cross_link