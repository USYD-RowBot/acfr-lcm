clear all; close all;
load ('../processed/nav_t.mat');

n=length(nav_t.PXF.imgname);
timestamp = zeros(n,1);
for ii=1:n
    timestamp(ii) = str2double(char(nav_t.PXF.imgname(ii)));
end

rt_file = load('./dive-vert/rt-pose.txt','r');
rt_utime1 = rt_file(:,1);
rt_utime2 = rt_file(:,2);
rt_x21 = rt_file(:,3:8)';

rtcam_file = load('./dive-vert/rt-campose.txt','r');
rtcam_utime1 = rtcam_file(:,1);
rtcam_utime2 = rtcam_file(:,2);
rtcam_x21 = rtcam_file(:,3:7)';

len = size (rt_x21, 2);
sigma = 1; model = 1; 
baseline = [];
error = [];
for ii =20:30
    b = trans2dm (rt_x21(1:3,ii));
    rt_x21_5dof = [b(1:2); rt_x21(4:6,ii)];
    baseline = [baseline b(3)];
    if 1
        rt_x21_new = [dm2trans([b(1:2); 0.6;]); rt_x21(4:6,ii)];
        [x_5dof, p21, S] = test_ba (sigma, model, rt_x21_new);
    else
        [x_5dof, p21, S] = test_ba (sigma, model, rt_x21(:,ii));
    end
    del = (p21-rt_x21_5dof)*RTOD'
    cov=sqrt(diag(S))*RTOD
    error = [error p21-rt_x21_5dof];
end

error = abs(error);

err_a = error(1,:)*RTOD; err_e = error(2,:)*RTOD;
err_r = error(3,:)*RTOD; err_p = error(4,:)*RTOD; err_h = error(5,:)*RTOD;

figure(1);
subplot (2,1,1); plot(err_a); grid on; title('error (ae)'); xlabel ('a');
subplot (2,1,2); plot(err_e); grid on; xlabel ('e');

figure(2);
subplot (3,1,1); plot(err_r); grid on; title('error (rph)'); xlabel ('r');
subplot (3,1,2); plot(err_p); grid on; xlabel ('p');
subplot (3,1,3); plot(err_h); grid on; xlabel ('h');

max_a = max (err_a)
max_e = max (err_e)
mean_a = mean (err_a)
mean_e = mean (err_e)


% error = [];
% for ii =1:len
%     rtidx = find (rt_utime1(ii) == rtcam_utime1 &...
%                   rt_utime2(ii) == rtcam_utime2);
% 
%     if (~isempty (rtidx))
%         p21 = test_ba (0, 1, rt_x21(:,ii));
%         error = [error p21-rtcam_x21(:,rtidx)]
%     end
% end