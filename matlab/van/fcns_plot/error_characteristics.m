function error_characteristics(field,TheJournal);


Nf       = TheJournal.Index.Nf;
Xp_i     = TheJournal.Index.Xp_i;
refind   = TheJournal.Index.Xf_ii{1}(Xp_i(1:3)); % reference image XYZ pose index
mu       = TheJournal.(field).mu;
Sigma    = TheJournal.(field).Sigma;
featLUT  = TheJournal.Index.featureLUT';
pathlen  = TheJournal.Pathlen.pathvec3(1:Nf);
distance = zeros(Nf,1);
determinant = zeros(Nf,1);
for ii=1:Nf;
  ind = TheJournal.Index.Xf_ii{ii}(Xp_i(1:3));
  delta = mu(ind) - mu(refind);
  distance(ii) = sqrt(delta'*delta);
  determinant(ii) = det(Sigma(ind,ind))^(1/3);
end;

fignum = gcf;
figure(fignum);
set(gca,'fontsize',14);
subplot(2,1,1);
plot(pathlen,determinant,'.');
grid on;
xlabel('Path length [m]');
ylabel('Determinant of XY Sub-block');
subplot(2,1,2);
plot(distance,determinant,'.');
grid on;
xlabel('Reference Image Distance [m]');
ylabel('Determinant of XY Sub-block');

figure(fignum+1);
set(gca,'fontsize',14);
scale = determinant/determinant(2);
scatter(pathlen,distance,5*scale,determinant,'filled');
grid on;
colorbar('horiz');
xlabel('Path length [m]');
ylabel('Reference Image Distance [m]');
