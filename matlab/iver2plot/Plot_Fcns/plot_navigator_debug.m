
function [] = plot_navigator_debug () 

%LCM = lcmlog_2011_05_31_01 % X_vs [0 0 0 0 0 pi]
%LCM = lcmlog_2011_05_31_03 % X_sv [0, 0, 0, 0, 0.0349, 3.14159265]
%LCM = lcmlog_2011_05_31_04 % X_sv [0, 0, 0, 0, 0.00349, 3.14159265]
%LCM = lcmlog_2011_05_31_05 % added nis
%LCM = lcmlog_2011_05_31_06 % added nis
%LCM = lcmlog_2011_05_31_07
%LCM = lcmlog_2011_05_31_08
%LCM = lcmlog_2011_05_31_11
%LCM = lcmlog_2011_05_31_12
%LCM = lcmlog_2011_05_31_13
%LCM = lcmlog_2011_05_31_14
%LCM = lcmlog_2011_05_31_15
%LCM = lcmlog_2011_05_31_16
%LCM = lcmlog_2011_05_31_19
%LCM = lcmlog_2011_05_31_20
%LCM = lcmlog_2011_05_31_21 % all sensors
%LCM = lcmlog_2011_05_31_22 % ms rates and dvl velocities
%LCM = lcmlog_2011_06_01_00
%LCM = lcmlog_2011_06_01_03
%LCM = lcmlog_2011_06_01_05
%LCM = lcmlog_2011_06_01_06 %open loop with mahal 0.999 on all sensors
%LCM = lcmlog_2011_06_01_07 %open loop with mahal 0.999 exept KVH, DVL where off (HUGE DT)
%LCM = lcmlog_2011_06_01_08 %open loop with mahal 0.999 exept KVH, DVL OFF
%LCM = lcmlog_2011_06_01_01
%LCM = lcmlog_2011_06_01_02 % Dive 42 no gps
%LCM = lcmlog_2011_06_01_03 % Dive 38 no gps
%LCM = lcmlog_2011_06_01_04 % Dive 36 no gps
%LCM = lcmlog_2011_06_01_06 % Dive 34 no gps (quite a bit of dvl drop out)
%LCM = lcmlog_2011_06_01_08 % Dive 33 no gps

st = LCM.NAVIGATOR.utime(1)/1e6;
f_ind = find(LCM.NAVIGATOR.mu(:,1) ~= 0, 1, 'first');
x_orig = LCM.NAVIGATOR.mu(f_ind, 1);
y_orig = LCM.NAVIGATOR.mu(f_ind, 2);

Sigma = reshape(LCM.NAVIGATOR.Sigma',12,12,[]);
x_std = sqrt(squeeze(Sigma(1,1,:)));
y_std = sqrt(squeeze(Sigma(2,2,:)));
z_std = sqrt(squeeze(Sigma(3,3,:)));
r_std = sqrt(squeeze(Sigma(4,4,:)));
p_std = sqrt(squeeze(Sigma(5,5,:)));
h_std = sqrt(squeeze(Sigma(6,6,:)));
u_std = sqrt(squeeze(Sigma(7,7,:)));
v_std = sqrt(squeeze(Sigma(8,8,:)));
w_std = sqrt(squeeze(Sigma(9,9,:)));
a_std = sqrt(squeeze(Sigma(10,10,:)));
b_std = sqrt(squeeze(Sigma(11,11,:)));
c_std = sqrt(squeeze(Sigma(12,12,:)));

%for i=[1:100:size(Sigma,3)], figure(99),imagesc(abs(rhomatrix(Sigma(:,:,i)))), axis equal tight, colorbar, drawnow, end

figure(1)
	plot(LCM.NAVIGATOR.mu(f_ind:end,2), LCM.NAVIGATOR.mu(f_ind:end, 1), '.-b');
hold on
    lat = LCM.GPSD.latitude*RTOD;
    lon = LCM.GPSD.longitude*RTOD;
    [x y] = ll2xy(lat, lon, ...
            repmat(LCM.NAVIGATOR.org_latitude(f_ind)*RTOD, length(LCM.GPSD.latitude), 1), ...
            repmat(LCM.NAVIGATOR.org_longitude(f_ind)*RTOD, length(LCM.GPSD.longitude), 1));
    plot(x, y, '.c');
    % plot uncertianty
    %quiver (LCM.NAVIGATOR.mu(f_ind:100:end,2), LCM.NAVIGATOR.mu(f_ind:100:end, 1), ...
    %        sin(LCM.NAVIGATOR.mu(f_ind:100:end,6)), cos(LCM.NAVIGATOR.mu(f_ind:100:end,6)), 0.5, 'm');
%    plot(LCM.NAVIGATOR.mu(36808,2)-y_orig, LCM.NAVIGATOR.mu(36808,1)-x_orig ,'.r','MarkerSize', 20);
hold off
    axis equal, grid on;

figure(2)
subplot(3,1,1)
    plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,1)-x_orig, 'b') 
    grid on, title('x'), axis tight;
    hold on
        plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,1)-x_orig + 3*x_std, 'r') 
        plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,1)-x_orig - 3*x_std, 'r') 
    hold off
subplot(3,1,2)
    plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,2)-y_orig, 'b')
    grid on, title('y'), axis tight;
    hold on
        plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,2)-y_orig + 3*y_std, 'r')
        plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,2)-y_orig - 3*y_std, 'r')
    hold off
subplot(3,1,3)
    plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, -LCM.NAVIGATOR.mu(1:end,3), 'b')
    grid on, title('z'), axis tight;
grid on

figure(4)
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, ...
    LCM.NAVIGATOR.mu(1:end,6)/3.14*180, 'b')
     title('heading mu');  grid on;

if 1
    
figure(5)
subplot(3,1,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.eta(1:end, 1)/3.14*180, 'b') 
    title('roll innov'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.eta(1:end, 2)/3.14*180, 'b') 
    title('pitch innov');  grid on;
hold off
subplot(3,1,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.eta(1:end, 3)/3.14*180, 'b') 
     title('heading innov');  grid on;
hold off     

end

figure(6)
subplot(3,1,1);
plot(LCM.NAVIGATOR.utime(1:100:end)/1e6-st, ...
    minimizedAngle( LCM.NAVIGATOR.mu(1:100:end,4) )/3.14*180, 'b')  
    title('roll mu'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR.utime(1:100:end)/1e6-st, ...
    minimizedAngle( LCM.NAVIGATOR.mu(1:100:end,5) )/3.14*180, 'b')
    title('pitch mu');  grid on;
hold off
subplot(3,1,3);
plot(LCM.NAVIGATOR.utime(1:100:end)/1e6-st, ...
    minimizedAngle( LCM.NAVIGATOR.mu(1:100:end,6) )/3.14*180, 'b')
     title('heading mu');  grid on;
hold off   

if 1

figure(7)
subplot(3,1,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
     minimizedAngle( -LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.z(1:end, 1))/3.14*180, 'b') 
    title('roll raw'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
     minimizedAngle( -LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.z(1:end, 2))/3.14*180, 'b') 
    title('pitch raw');  grid on;
subplot(3,1,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
    minimizedAngle((LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.z(1:end, 3) + pi))/3.14*180, 'b') 
     title('heading raw');  grid on;
end

% heading raw-mu
figure(15)
heading_mu_interp = interp1(LCM.NAVIGATOR.utime(1:end)/1e6-st, ...
                LCM.NAVIGATOR.mu(1:end,6), ...
                LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, 'linear');
plot (LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
    minimizedAngle((LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.z(1:end, 3) + pi) -  heading_mu_interp)/3.14*180);

title('heading mu-raw'); grid on;

if 1

figure(8)
subplot(3,1,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.eta(1:end, 1), 'b') 
    title('u innov'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.eta(1:end, 2), 'b') 
    title('v innov');  grid on;
hold off
subplot(3,1,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.eta(1:end, 3), 'b') 
     title('w innov');  grid on;
hold off        
     
figure(9)
subplot(3,1,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.z(1:end, 1), 'b') 
    title('u raw'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.z(1:end, 2), 'b') 
    title('v raw');  grid on;
subplot(3,1,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.z(1:end, 3), 'b') 
     title('w raw');  grid on;     
  
end

figure(11)
subplot(3,1,1);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,7), 'b')  
    title('u mu'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,8), 'b')
    title('v mu');  grid on;
hold off
subplot(3,1,3);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,9), 'b')
     title('w mu');  grid on;
hold off    

figure(12)
plot(LCM.NAVIGATOR_PRED_DEBUG.utime(1:end)/1e6-st, ...
    LCM.NAVIGATOR_PRED_DEBUG.dt(1:end), 'b');
    title('dt');  grid on;

figure(13)
subplot(3,1,1);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, u_std, 'b')  
    title('u std'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, v_std, 'b')
    title('v std');  grid on;
hold off
subplot(3,1,3);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, w_std, 'b')
     title('w std');  grid on;
hold off 
    
figure(14)
subplot(3,1,1);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,10)/3.14*180, 'b')  
    title('p mu'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,11)/3.14*180, 'b')
    title('q mu');  grid on;
hold off
subplot(3,1,3);
plot(LCM.NAVIGATOR.utime(1:end)/1e6-st, LCM.NAVIGATOR.mu(1:end,12)/3.14*180, 'b')
     title('r mu');  grid on;
hold off   

if 1
figure(15)   
subplot(2,1,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.z(1:end, 1)/3.14*180, 'b') 
    title('KVH r raw'); grid on;
subplot(2,1,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.eta(1:end, 1)/3.14*180, 'b') 
     title('KVH r innov');  grid on;
end

figure(16)
subplot(3,1,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.eta(1:end, 1), 'b') 
    title('MS PQR p innov'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.eta(1:end, 2), 'b') 
    title('MS PQR q innov');  grid on;
hold off
subplot(3,1,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.eta(1:end, 3), 'b') 
     title('MS PQR r innov');  grid on;
hold off        
     
figure(17)
subplot(3,1,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.z(1:end, 1), 'b') 
    title('MS PQR p raw'); grid on;
subplot(3,1,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.z(1:end, 2), 'b') 
    title('MS PQR q raw');  grid on;
subplot(3,1,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.z(1:end, 3), 'b') 
     title('MS PQR r raw');  grid on;     


figure(50)
subplot(3,1,1);
lvl999 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end))) * chi2inv(0.999,3);
lvl99 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end))) * chi2inv(0.99,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.nis(1:end, 1), 'b') 
hold on
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, lvl99, 'c');
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_RDI_PD4_UVW.utime(1:end)/1e6-st, lvl999, 'r');
hold off
title('RDI UVW NIS'); grid on;

subplot(3,1,2);
lvl999 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end))) * chi2inv(0.999,3);
lvl99 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end))) * chi2inv(0.99,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.nis(1:end, 1), 'b') 
hold on
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, lvl99, 'c');
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_THETAPHIPSI.utime(1:end)/1e6-st, lvl999, 'r');
hold off
title('MS RPH NIS'); grid on;

if 1
subplot(3,1,3);
lvl999 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.utime(1:end))) * chi2inv(0.999,1);
lvl99 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.utime(1:end))) * chi2inv(0.99,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.nis(1:end, 1), 'b') 
hold on
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.utime(1:end)/1e6-st, lvl99, 'c');
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_KVH_DSP3000_R.utime(1:end)/1e6-st, lvl999, 'r');
hold off
title('KVH R NIS'); grid on;
end

figure(51)
subplot(3,1,1);
lvl999 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_DSTAR_SSP1_Z.utime(1:end))) * chi2inv(0.999,1);
lvl99 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_DSTAR_SSP1_Z.utime(1:end))) * chi2inv(0.99,1);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_DSTAR_SSP1_Z.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_DSTAR_SSP1_Z.nis(1:end, 1), 'b') 
hold on
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_DSTAR_SSP1_Z.utime(1:end)/1e6-st, lvl99, 'c');
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_DSTAR_SSP1_Z.utime(1:end)/1e6-st, lvl999, 'r');
hold off
title('DS Z NIS'); grid on;


subplot(3,1,2);
lvl999 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end))) * chi2inv(0.999,3);
lvl99 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end))) * chi2inv(0.99,3);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.nis(1:end, 1), 'b') 
hold on
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, lvl99, 'c');
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_MS_GX1_ABC.utime(1:end)/1e6-st, lvl999, 'r');
hold off
title('MS PQR NIS'); grid on;

subplot(3,1,3);
lvl999 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_GPSD_XY.utime(1:end))) * chi2inv(0.999,2);
lvl99 = ones(1,length(LCM.NAVIGATOR_MEAS_DEBUG_OMF_GPSD_XY.utime(1:end))) * chi2inv(0.99,2);
plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_GPSD_XY.utime(1:end)/1e6-st, ...
     LCM.NAVIGATOR_MEAS_DEBUG_OMF_GPSD_XY.nis(1:end, 1), 'b') 
hold on
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_GPSD_XY.utime(1:end)/1e6-st, lvl99, 'c');
    plot(LCM.NAVIGATOR_MEAS_DEBUG_OMF_GPSD_XY.utime(1:end)/1e6-st, lvl999, 'r');
hold off
title('GSP XY NIS'); grid on;


end

function angle = minimizedAngle( angle)

for n = 1:length(angle(:))
    while angle(n) < -pi
      angle(n) = angle(n) + 2*pi;
    end
    while angle(n) >= pi
      angle(n) = angle(n) - 2*pi;
    end
end

end
