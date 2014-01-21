Ccycles = 1;
Scycles = 1;

load /files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112c/results.mat;

Naug   = TheJournal.Index.Naug;
Nv     = TheJournal.Index.Nv;
Xa_i   = TheJournal.Index.Xa_i;
Xp_i   = TheJournal.Index.Xp_i;
A      = TheJournal.Eif.Lambda(Xa_i,Xa_i);

Lambda = A;
for ii=1:6;
  Lambda = blkdiag(Lambda,A);
end

N = length(Lambda)/12
eta = randn(length(Lambda),1);

tic; mu_t = Lambda\eta; disp(toc);

mu_o = randn(length(Lambda),1);

profile on;
tic;
mu_e = multigrid(Lambda,eta,mu_o,Nv,Xp_i,Ccycles,Scycles);
disp(toc);
profile report;

%figure(10);
%plot(1:Naug,mu_o-mu_t,'b',1:Naug,mu_e-mu_t,'r');
