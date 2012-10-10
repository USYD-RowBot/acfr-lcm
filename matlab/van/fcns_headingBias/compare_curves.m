load compass_bias.op.mat;
Pop = P;

load compass_bias.phins2.mat;
Pp5 = P;
Pp5(end) = Pop(end);

hdg_rdi = [0:1:360]'*DTOR;

hdg_op = compass_correction(hdg_rdi,Pop);
hdg_p5 = compass_correction(hdg_rdi,Pp5);

figure(1); clf;
hold on;
plot(hdg_rdi*RTOD,hdg_op*RTOD,'g-','linewidth',1);
plot(hdg_rdi*RTOD,hdg_p5*RTOD,'m-','linewidth',1);
plot(hdg_rdi*RTOD,hdg_rdi*RTOD,'k--','linewidth',1);
hold off;
legend('image','phins','1:1',0);
grid on;
axis equal;
xlabel('rdi heading [degrees]');
ylabel('corrected heading [degrees]');
title('comparison of image and phins derived heading bias corrections');
