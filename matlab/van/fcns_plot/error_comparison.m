clear all;

% JHU-6 nav
load /files1/processed/van/output/jhu04-6_gridsurvey3/nav_t.mat;

% DR result
tmpDR = load('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20050316a_DR_2000:10:3000/results.mat')

% VAN result
tmpVAN = load('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20050209a_2000:10:3000/results.mat')

% camera samples index
ii = nav_t.PXF.camind.RDI(2000:10:3000);
N = length(ii);

% integrated XY path length
t  = nav_t.RDI.rovtime;
px = diff(nav_t.RDI.nx);
py = diff(nav_t.RDI.ny);
pathlen = cumsum((px.^2 + py.^2).^0.5);
% take care of offset
t  = t - t(ii(1));
pathlen = pathlen - pathlen(ii(1));
pathlen(pathlen==0) = nan;


% distance away from reference node
xo = nav_t.RDI.nx(ii(1));
yo = nav_t.RDI.ny(ii(1));
dx = nav_t.RDI.nx-xo;
dy = nav_t.RDI.ny-yo;
distance = (dx.^2 + dy.^2).^0.5;
distance(distance==0) = nan;

[detDR,detVAN] = deal(zeros(N,1));
for k=1:tmpDR.TheJournal.Index.fn+1
  Xf_i = tmpDR.TheJournal.Index.Xf_ii{k};
  %detDR(k) = trace(tmpDR.TheJournal.Ekf.Sigma(Xf_i(1:2),Xf_i(1:2)));
  %detVAN(k) = trace(tmpVAN.TheJournal.Ekf.Sigma(Xf_i(1:2),Xf_i(1:2)));
  detDR(k) = det(tmpDR.TheJournal.Ekf.Sigma(Xf_i(1:2),Xf_i(1:2))).^0.5;
  detVAN(k) = det(tmpVAN.TheJournal.Ekf.Sigma(Xf_i(1:2),Xf_i(1:2))).^0.5;
end;

% plot DR trajectory
figure(1); clf;
plotTraj(tmpDR.TheJournal,nav_t,2000,3000,tmpDR.TheConfig);
% plot VAN trajectory
figure(2); clf;
plotTraj(tmpVAN.TheJournal,nav_t,2000,3000,tmpVAN.TheConfig);

% plot path length vs. time
figure(3); clf;
plot(t(ii),pathlen(ii),'k');
xlabel('Time [s]');
ylabel('Path Length [m]');
title('Vehicle Travel vs. Time @ 5cm/s Prescribed Forward Velocity');

% plot uncertainty vs. path length
figure(4); clf;
[ax,h1,h2] = plotyy(t(ii),detDR,t(ii),detVAN);
set(h1,'marker','.','linestyle','none','color','b');
set(h2,'marker','.','linestyle','none','color','r');
xlabel('Misson Time [s]');
ylabel('XY Determinant');
title('Comparison of Error Growth as a Function of Time');
legend([h1,h2],'DR','VAN');

% plot uncertainty vs. network distance
figure(5); clf;
[ax,h1,h2] = plotyy(distance(ii),detDR,distance(ii),detVAN);
set(h1,'marker','.','linestyle','none','color','b');
set(h2,'marker','.','linestyle','none','color','r');
xlabel('Distance Away from Reference Image [m]');
ylabel('XY Determinant');
title('Comparison of Error Growth as a Function of Network Distance');
legend([h1;h2],'DR','VAN');

% plot uncertainty vs. time
figure(6); clf;
[ax,h1,h2] = plotyy(pathlen(ii),detDR,pathlen(ii),detVAN);
set(h1,'marker','.','linestyle','none','color','b');
set(h2,'marker','.','linestyle','none','color','r');
xlabel('Integrated Path Length [m]');
ylabel('XY Determinant');
title('Comparison of Error Growth as a Function of Path Length');
legend([h1,h2],'DR','VAN');
