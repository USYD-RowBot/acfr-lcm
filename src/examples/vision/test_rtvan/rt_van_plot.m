% compare rt-van and matlab's pos prior

clear all; close all;
load ('./nav_t.mat');

n=length(nav_t.PXF.imgname);
timestamp = zeros(n,1);
for ii=1:n
    timestamp(ii) = str2double(char(nav_t.PXF.imgname(ii)));
end

rt_file = load('./dive-horz/rt-pose.txt','r');
rt_utime1 = rt_file(:,1);
rt_utime2 = rt_file(:,2);
rt_x21 = rt_file(:,3:8)';

rtv_file = load('./dive-horz/rt-vpose.txt','r');
rtv_utime = rtv_file(:,1);
rtv_xlv = rtv_file(:,2:7)';

rtcam_file = load('./dive-horz/rt-campose.txt','r');
rtcam_utime1 = rtcam_file(:,1);
rtcam_utime2 = rtcam_file(:,2);
rtcam_x21 = rtcam_file(:,3:7)';

mat_file = load('./dive-horz/matlab-pose.txt','r');
mat_utime1 = mat_file(:,1);
mat_utime2 = mat_file(:,2);
mat_x21 = mat_file(:,3:8)';
mat_v21 = mat_file(:,9:14)';

matcam_file = load('./dive-horz/matlab-campose.txt','r');
matcam_utime1 = mat_file(:,1);
matcam_utime2 = mat_file(:,2);
matcam_x21 = mat_file(:,3:7)';

len = size(mat_file,1);
error = []; err_idx = 1;
campitch = []; mat_c_idx = []; rt_c_idx = [];
for ii=1:len
    rtidx = find (mat_utime1(ii) == rt_utime1 &...
                  mat_utime2(ii) == rt_utime2);
    if (~isempty (rtidx))
        error(:,err_idx) = mat_x21(:,ii) - rt_x21(:,rtidx);
        error(4:6,err_idx) = error(4:6,err_idx) * RTOD;
        err_idx = err_idx + 1;
        mat_c_idx = [mat_c_idx; ii];
        rt_c_idx = [rt_c_idx; rtidx];
        
        pitch_idx = find(timestamp == mat_utime1(ii));
        if (~isempty(pitch_idx))
            campitch = [campitch; nav_t.PXF.encoderpitch(pitch_idx)];
        end
    end
end

% figure(1);
% plot(error(1,:)); grid on; title ('xyz');
% hold on; plot(error(2,:),'r');  plot(error(3,:),'g'); hold off; 
% legend(['x';'y';'z'])
% 
% figure(2); 
% plot(error(4,:)); grid on; title ('rph (deg)');
% hold on; plot(error(5,:),'r');  plot(error(6,:),'g'); hold off;
% legend(['r';'p';'h'])

figure(1);
subplot(3,1,1); plot(mat_x21(1,mat_c_idx)); hold on; plot(rt_x21(1,rt_c_idx),'r'); 
plot(campitch,'g'); plot(mat_x21(1,mat_c_idx)-rt_x21(1,rt_c_idx),'k');
grid on; title ('nav prior matlab vs. rt-van (c21 xyz)'); xlabel ('x_c21'); legend(['mat';'rt ';'pit';'err';])
subplot(3,1,2); plot(mat_x21(2,mat_c_idx)); hold on; plot(rt_x21(2,rt_c_idx),'r'); grid on; xlabel ('y_c21'); legend(['mat';'rt ';])
subplot(3,1,3); plot(mat_x21(3,mat_c_idx)); hold on; plot(rt_x21(3,rt_c_idx),'r'); grid on; xlabel ('z_c21'); legend(['mat';'rt ';])

figure(2); 
subplot(3,1,1); plot(mat_x21(4,mat_c_idx)*RTOD); hold on; plot(rt_x21(4,rt_c_idx)*RTOD,'r'); grid on; 
title ('v21 rph'); xlabel ('r_c21'); legend(['mat';'rt ';])
subplot(3,1,2); plot(mat_x21(5,mat_c_idx)*RTOD); hold on; plot(rt_x21(5,rt_c_idx)*RTOD,'r'); grid on; xlabel ('p_c21'); legend(['mat';'rt ';])
subplot(3,1,3); plot(mat_x21(6,mat_c_idx)*RTOD); hold on; plot(rt_x21(6,rt_c_idx)*RTOD,'r'); grid on; xlabel ('h_c21'); legend(['mat';'rt ';])


%% camera
error_cam = []; errcam_idx = 1;
campitch = []; mat_5dof = []; rt_c_idx = []; heading = [];

chidx = find (rtcam_x21(1,:) < -150*DTOR);
rtcam_x21(1,chidx) = rtcam_x21(1,chidx) + 360*DTOR;

cmp_rt = 0; % 0 to compare with matlab cam meas

for ii=1:len
    if (cmp_rt)
        rtidx = find (mat_utime1(ii) == rtcam_utime1 &...
                      mat_utime2(ii) == rtcam_utime2);
        if (~isempty (rtidx))
            b = trans2dm (mat_x21(1:3,ii));
            mat_x21_5dof = [b(1:2); mat_x21(4:6,ii)];
            error_cam(:,errcam_idx) = (mat_x21_5dof - rtcam_x21(:,rtidx))*RTOD;
            if (error_cam(1,errcam_idx) > 180)
                error_cam(1,errcam_idx) = error_cam(1,errcam_idx) - 360;
            end
            errcam_idx = errcam_idx + 1;
            if (mat_x21_5dof(1) > 180*DTOR)
                mat_x21_5dof(1) = mat_x21_5dof(1) - 360*DTOR;
            elseif (mat_x21_5dof(1) < - 150*DTOR)
                mat_x21_5dof(1) = mat_x21_5dof(1) + 360*DTOR;
            end
            mat_5dof = [mat_5dof mat_x21_5dof];
            rt_c_idx = [rt_c_idx; rtidx];

            % pitch
            pitch_idx = find(timestamp == mat_utime1(ii));
            if (~isempty(pitch_idx))
                campitch = [campitch; nav_t.PXF.encoderpitch(pitch_idx)];

                % heading
                [v, heading_idx] = min (abs(nav_t.PXF.rovtime(pitch_idx) - nav_t.RDI.rovtime));
                heading = [heading; nav_t.RDI.heading(heading_idx)];
            end
        end
    else
        % matlab cam
        matidx = find(mat_utime1(ii) == matcam_utime1 &...
                      mat_utime2(ii) == matcam_utime2);

        if (~isempty(matidx))
            b = trans2dm (mat_x21(1:3,ii));
            mat_x21_5dof = [b(1:2); mat_x21(4:6,ii)];
            error_cam(:,errcam_idx) = (mat_x21_5dof - matcam_x21(:,matidx))*RTOD;
            if (error_cam(1,errcam_idx) > 180)
                error_cam(1,errcam_idx) = error_cam(1,errcam_idx) - 360;
            end
            errcam_idx = errcam_idx + 1;
            if (mat_x21_5dof(1) > 180*DTOR)
                mat_x21_5dof(1) = mat_x21_5dof(1) - 360*DTOR;
            elseif (mat_x21_5dof(1) < - 150*DTOR)
                mat_x21_5dof(1) = mat_x21_5dof(1) + 360*DTOR;
            end
            mat_5dof = [mat_5dof mat_x21_5dof];
            mat_c_idx = [mat_c_idx; matidx];

            % pitch
            pitch_idx = find(timestamp == mat_utime1(ii));
            if (~isempty(pitch_idx))
                campitch = [campitch; nav_t.PXF.encoderpitch(pitch_idx)];

                % heading
                [v, heading_idx] = min (abs(nav_t.PXF.rovtime(pitch_idx) - nav_t.RDI.rovtime));
                heading = [heading; nav_t.RDI.heading(heading_idx)];
            end
        end
    end
end

if (cmp_rt)
    % figure(3); plot(error_cam'); grid on; legend(['a'; 'e'; 'r'; 'p'; 'h'])
    figure(3);
    subplot(3,1,1); plot(mat_5dof(1,:)*RTOD); hold on; plot(rtcam_x21(1,rt_c_idx)*RTOD,'r'); grid on;
    title('matlab dr vs cam (a,e)'); xlabel ('a'); legend(['mat';'cam';])
    subplot(3,1,2); plot(mat_5dof(2,:)*RTOD); hold on; plot(rtcam_x21(2,rt_c_idx)*RTOD,'r'); grid on; xlabel ('e'); legend(['mat';'cam';])
    subplot(3,1,3); plot(heading*RTOD); grid on; xlabel('heading');

    figure(4);
    subplot(3,1,1); plot(mat_5dof(3,:)*RTOD); hold on; plot(rtcam_x21(3,rt_c_idx)*RTOD,'r'); grid on;
    title('dr vs cam r,p,h');  xlabel ('r'); legend(['mat';'cam';])
    subplot(3,1,2); plot(mat_5dof(4,:)*RTOD); hold on; plot(rtcam_x21(4,rt_c_idx)*RTOD,'r'); grid on; xlabel ('p'); legend(['mat';'cam';])
    subplot(3,1,3); plot(mat_5dof(5,:)*RTOD); hold on; plot(rtcam_x21(5,rt_c_idx)*RTOD,'r');
    plot(campitch*RTOD,'g'); grid on; xlabel ('h'); legend(['mat';'cam';'pit';])
else
    figure(3);
    subplot(3,1,1); plot(mat_5dof(1,:)*RTOD); hold on; plot(matcam_x21(1,mat_c_idx)*RTOD,'r'); grid on;
    title('matlab dr vs cam (a,e)'); xlabel ('a'); legend(['mat';'cam';])
    subplot(3,1,2); plot(mat_5dof(2,:)*RTOD); hold on; plot(matcam_x21(2,mat_c_idx)*RTOD,'r'); grid on; xlabel ('e'); legend(['mat';'cam';])
    subplot(3,1,3); plot(heading*RTOD); grid on; xlabel('heading');

    figure(4);
    subplot(3,1,1); plot(mat_5dof(3,:)*RTOD); hold on; plot(matcam_x21(3,mat_c_idx)*RTOD,'r'); grid on;
    title('dr vs cam r,p,h');  xlabel ('r'); legend(['mat';'cam';])
    subplot(3,1,2); plot(mat_5dof(4,:)*RTOD); hold on; plot(matcam_x21(4,mat_c_idx)*RTOD,'r'); grid on; xlabel ('p'); legend(['mat';'cam';])
    subplot(3,1,3); plot(mat_5dof(5,:)*RTOD); hold on; plot(matcam_x21(5,mat_c_idx)*RTOD,'r');
    plot(campitch*RTOD,'g'); grid on; xlabel ('h'); legend(['mat';'cam';'pit';])
end
% figure(5);
% subplot(2,1,1); plot(mod(abs(mat_5dof(1,:)-rtcam_x21(1,rt_c_idx))*RTOD,360)); grid on; 
% title ('error in a,e'); xlabel ('error in a');
% subplot(2,1,2); plot(mod(abs(mat_5dof(2,:)-rtcam_x21(2,rt_c_idx))*RTOD,360)); grid on; xlabel ('error in e');
% 
% figure(6);
% plot(mod(abs(mat_5dof(3,:)-rtcam_x21(3,rt_c_idx))*RTOD,360)); grid on;
% hold on; plot(mod(abs(mat_5dof(4,:)-rtcam_x21(4,rt_c_idx))*RTOD,360),'r'); title ('error in r,p'); legend(['r';'p'])

%% compare camera results (rt vs. matlab)
matcamlen = size(matcam_file,1);
error = []; err_idx = 1;

for ii=1:matcamlen
    rtidx = find (matcam_utime1(ii) == rtcam_utime1 &...
                  matcam_utime2(ii) == rtcam_utime2);
    if (~isempty (rtidx))
        error(:,err_idx) = matcam_x21(:,ii) - rtcam_x21(:,rtidx);
        error = error * RTOD;
        rt_c_idx = [rt_c_idx; rtidx];
    end
end

figure(5);
subplot(2,1,1); plot(matcam_x21(1,mat_c_idx)); hold on; plot(rtcam_x21(1,rt_c_idx),'r'); grid on; title ('cam matlab vs. rt-van (a,e)'); xlabel ('a'); legend(['mat';'rt ';])
subplot(2,1,2); plot(matcam_x21(2,mat_c_idx)); hold on; plot(rtcam_x21(2,rt_c_idx),'r'); grid on; xlabel ('e'); legend(['mat';'rt ';])

figure(6); 
subplot(3,1,1); plot(matcam_x21(3,mat_c_idx)*RTOD); hold on; plot(rtcam_x21(3,rt_c_idx)*RTOD,'r'); grid on; title ('camera matlab vs. rtvan rph'); xlabel ('r'); legend(['mat';'rt ';])
subplot(3,1,2); plot(matcam_x21(4,mat_c_idx)*RTOD); hold on; plot(rtcam_x21(4,rt_c_idx)*RTOD,'r'); grid on; xlabel ('p'); legend(['mat';'rt ';])
subplot(3,1,3); plot(matcam_x21(5,mat_c_idx)*RTOD); hold on; plot(rtcam_x21(5,rt_c_idx)*RTOD,'r'); grid on; xlabel ('h'); legend(['mat';'rt ';])


if 0
%% vehicle
errorv = []; errv_idx = 1;
campitch = []; mat_v_idx = []; rt_v21 = [];

for ii=1:len
    [v, x_v1idx] = min (abs(mat_utime1(ii) - rtv_utime));
    [v, x_v2idx] = min (abs(mat_utime2(ii) - rtv_utime));
    

    x_v1 = rtv_xlv (:,x_v1idx);
    x_v2 = rtv_xlv (:,x_v2idx);
    x_v21 = tail2tail (x_v2, x_v1);

    errorv(:,errv_idx) = mat_v21(:,ii) - x_v21;
    errorv(4:6,errv_idx) = errorv(4:6,errv_idx) * RTOD;
    errv_idx = errv_idx + 1;
    mat_v_idx = [mat_v_idx; ii];
    rt_v21 = [rt_v21 x_v21];

    pitch_idx = find(timestamp == mat_utime1(ii));
    if (~isempty(pitch_idx))
        campitch = [campitch; nav_t.PXF.encoderpitch(pitch_idx)];
    end
end

% figure(4); plot(errorv(1:3,:)'); grid on; legend(['x'; 'y'; 'z';]); xlabel ('xyz v21'); xlim([0,500])
% figure(5); plot(errorv(4:6,:)'); grid on; legend(['r'; 'p'; 'h']); xlabel ('rph v21'); xlim([0,500])

figure(5);
subplot(3,1,1); plot(mat_v21(1,mat_v_idx)); hold on; plot(rt_v21(1,:),'r'); 
plot(campitch,'g'); plot(mat_v21(1,mat_v_idx)-rt_v21(1,:),'k');
grid on; xlabel ('x_v21'); legend(['mat';'rt ';'pit';'err';]); %xlim([0,500])
subplot(3,1,2); plot(mat_v21(2,mat_v_idx)); hold on; plot(rt_v21(2,:),'r'); grid on; xlabel ('y_v21'); legend(['mat';'rt ';]); %xlim([0,500])
subplot(3,1,3); plot(mat_v21(3,mat_v_idx)); hold on; plot(rt_v21(3,:),'r'); grid on; xlabel ('z_v21'); legend(['mat';'rt ';]); %xlim([0,500])

figure(6);
subplot(3,1,1); plot(mat_v21(4,mat_v_idx)); hold on; plot(rt_v21(4,:),'r'); grid on; xlabel ('r_v21'); legend(['mat';'rt ';]); %xlim([0,500])
subplot(3,1,2); plot(mat_v21(5,mat_v_idx)); hold on; plot(rt_v21(5,:),'r'); grid on; xlabel ('p_v21'); legend(['mat';'rt ';]); %xlim([0,500])
subplot(3,1,3); plot(mat_v21(6,mat_v_idx)); hold on; plot(rt_v21(6,:),'r'); grid on; xlabel ('h_v21'); legend(['mat';'rt ';]); %xlim([0,500])
end