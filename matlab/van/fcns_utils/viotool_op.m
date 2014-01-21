% camera survey design curves
% OP 20020412
% figure(1) has two charts for overlap along direction of motion
% pick a speed on the chart on the right, go up to the flash
% period, find the mpf (meters per flash), move across to the chart
% on the left and for the same mpf find the overlap for a given
% altitude.
% figure(2) is a chart for overlap across tracklines

DTR = pi/180;
% FOV
FOVx = 42.4*DTR; % x axis across tracklines
FOVy = 34.5*DTR; % y axis along direction of motion

% speed range
speed_vec = [0.05:0.05:0.75]; %[m/s]
% altitude range
alt_vec = [0.5:0.05:4]; %[m]
% strobe period range
t_vec = [1:0.25:4]; %[s] used on contour lines

% meters per flash range
max_mpf = 2*max(alt_vec)*tan(FOVy/2);
mpf_vec = [0:0.1:max_mpf];

% calculate overlap as a function of speed, altitude and strobe
% period
% first: calculate Meters per Flash (mpf) as a function of speed
% and strobe period
t_mat = mpf_vec'*(speed_vec.^-1);


% calculate overlap fraction as a function of meters per flash and
% altitude

footprint_vec = 2*alt_vec*tan(FOVy/2); % along direction of motion

mpf_mat = mpf_vec'*ones(size(footprint_vec));
foot_mat = ones(size(mpf_vec))'*footprint_vec;

ovlp_mat = (foot_mat-mpf_mat)./foot_mat;

% since overlap can't be less than zero make negative overlaps 0
ovlp_mat(ovlp_mat < 0) = 0;


figure(1);
subplot(1,2,1);
[cs,h] = contour(alt_vec,mpf_vec,ovlp_mat);
clabel(cs,h);
xlabel('altitude [m]');
ylabel('meters per flash [m]');
title('Along track overlap contours');
grid on;

subplot(1,2,2);
[cs, h] = contour(speed_vec,mpf_vec,t_mat,t_vec);
clabel(cs,h);
xlabel('speed [m/s]');
ylabel('meters per flash [m]');
title('Strobe period contours [s]');
grid on;

% calculate across track overlap based on trackline spacing
max_footx = 2*max(alt_vec)*tan(FOVx/2);
track_vec = [0:0.1:max_footx]; % trackline spacing vector

footprintx_vec = 2*alt_vec*tan(FOVx/2);
track_mat = track_vec'*ones(size(footprintx_vec));

footx_mat = ones(size(track_vec))'*footprintx_vec;

track_ovlp_mat = (footx_mat - track_mat)./footx_mat;

% since overlap can't be less than zero make negative overlaps 0
track_ovlp_mat(track_ovlp_mat < 0) = 0;


figure(2)
[cs,h] = contour(alt_vec,track_vec,track_ovlp_mat);
clabel(cs,h);
xlabel('altitude [m]');
ylabel('trackline spacing [m]');
title('Cross-track overlap contours');
grid on;
