function cpose = rme2op(TheJournal,TheConfig);

mu = TheJournal.Eif.Lambda\TheJournal.Eif.eta;

Xp_i = TheJournal.Index.Xp_i;


x_wl = TheConfig.SensorXform.x_wl;
x_vc = TheConfig.SensorXform.PXF.x_vs;

% camera poses
cpose = zeros(6,TheJournal.Index.Nf);
for ii=1:TheJournal.Index.Nf;
  x_lvi = mu(TheJournal.Index.Xf_ii{ii}(Xp_i));
  x_lci = head2tail(x_lvi,x_vc);
  x_wci = head2tail(x_wl,x_lci);
  cpose(:,ii) = x_wci([4:6,1:3]); % OP assumes [r,p,h,x,y,z]
end;
