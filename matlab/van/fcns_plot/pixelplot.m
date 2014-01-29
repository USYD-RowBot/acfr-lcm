function pixelplot(u,v,bnum,udata,vdata,fignum)
% helper function used in find_dvl_beams_in_camera_fov.m
% 20040319 rme

figure(fignum); clf;
hold on;
cvec = ['r';'b';'g';'m'];
rvec = ['r1';'r2';'r3';'r4'];
rr = [];
for ii=1:4
  bb = find(bnum==ii);
  if ~isempty(bb)
    rr = [rr; ii];
    plot(u(bb),v(bb),'.','Color',cvec(ii));
  end
end
hold off;
legend(rvec(rr,:),1);
grid on;
axis equal ij;
width = udata(2)-udata(1);
height= vdata(2)-vdata(1);
rectangle('Position',[udata(1), vdata(1), width, height]);
