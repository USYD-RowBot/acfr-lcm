tic;
Rchol = chol(TheJournal.Eif.Lambda);
mu_hat = pcg(TheJournal.Eif.Lambda,TheJournal.Eif.eta,[],[],Rchol',Rchol,TheJournal.Eif.mu);
fprintf('PCG: dt = %.2e\n',toc);

tic;
Rcholinc = cholinc(TheJournal.Eif.Lambda,1e-3);
mu_hat = pcg(TheJournal.Eif.Lambda,TheJournal.Eif.eta,[],[],Rcholinc',Rcholinc,TheJournal.Eif.mu);
fprintf('PCG: dt = %.2e\n',toc);
