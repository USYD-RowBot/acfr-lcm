if ~exist('Sigma','var') 
  Lambda = TheJournal.Eif.Lambda;
  Sigma = full(Lambda^-1);
  Rchol = cholinc(TheJournal.Eif.Lambda,1e-3);
end

fni  = 25;
Xf_i = TheJournal.Index.Xf_ii{fni};
Xp_i = TheJournal.Index.Xp_i;
Naug = TheJournal.Index.Naug;
Np   = TheJournal.Index.Np;

Icol = spalloc(Naug,Np,Np);
Icol(Xf_i(Xp_i),:) = speye(Np);

Sigma(Xf_i(Xp_i),Xf_i(Xp_i))

Sigma_coli = zeros(Naug,Np);
switch 2
case 0
 Sigma_coli = full(Lambda \ Icol);
 Sigma_coli(Xf_i(Xp_i),:) 
case 1
 tic
 for ii=1:Np
   TOL = 1e-6;
   MAXIT = 30;
   Scol_est = zeros(Naug,1);
   Scol_est(1:ii-1) = Sigma_coli(ii,1:ii-1);
   Sigma_coli(:,ii) = pcg(TheJournal.Eif.Lambda,Icol(:,ii),TOL,MAXIT,Rchol',Rchol,Scol_est);
 end
 toc
 Sigma_coli(Xf_i(Xp_i),:)
case 2
 tic
 Sigma_coli = full(Lambda \ Icol);
 toc
 Sigma_coli(Xf_i(Xp_i),:) 
case 3
 xxi = Xf_i(Xp_i);
 mpi = markov_blanket(Lambda,xxi);
 mmi = 1:Naug;
 mmi([xxi,mpi]) = [];
 Lambda_alpha = Lambda(mpi,mpi) - Lambda(mpi,xxi)*Lambda(xxi,xxi)^-1*Lambda(xxi,mpi);
 Nmp = length(mpi);
 Nmm = length(mmi);

 A = [Lambda(xxi,xxi),   Lambda(xxi,mpi),    spalloc(6,Nmm,0); ...
      spalloc(Nmp,6,0),  Lambda_alpha   ,    Lambda(mpi,mmi); ...
      spalloc(Nmm,6,0),  spalloc(Nmm,Nmp,0), Lambda(mmi,mmi) - Lambda(mmi,mpi)*Lambda_alpha^-1*Lambda(mpi,mmi)];

 B = [ eye(Np); ...
      -Lambda(mpi,xxi)*Lambda(xxi,xxi)^-1; ...
       Lambda(mmi,mpi)*Lambda_alpha^-1*Lambda(mpi,xxi)*Lambda(xxi,xxi)^-1];

 tic;
 Sigma_coli = full(A\B);
 toc
 Sigma_coli(1:6,:)
end
