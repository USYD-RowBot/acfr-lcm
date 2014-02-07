function testMG(Ccycles,Scycles)

load /files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112c/results.mat;

Naug   = TheJournal.Index.Naug;
Nv     = TheJournal.Index.Nv;
Xa_i   = TheJournal.Index.Xa_i;
Xp_i   = TheJournal.Index.Xp_i;
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);
eta    = TheJournal.Eif.eta(Xa_i);
mu_o   = TheJournal.Eif.mu(Xa_i);
mu_t   = Lambda \ eta;

profile on;
mu_e = multigrid(Lambda,eta,mu_o,Nv,Xp_i,Ccycles,Scycles);
profile report;

figure(10);
plot(1:Naug,mu_o-mu_t,'b',1:Naug,mu_e-mu_t,'r');
