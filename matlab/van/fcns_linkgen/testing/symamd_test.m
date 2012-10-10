
% construct adjacency matrix
Nf = TheJournal.Index.Nf;
Nv = TheJournal.Index.Nv;
Naug = TheJournal.Index.Naug;
Amatrix =  ( (link_t.vlinks==1) + speye(Nf) + spdiags(repmat([1,0,1],Nf,1),-1:1,Nf,Nf) );
Amatrix(Nf,Nf+1)   = 1;
Amatrix(Nf+1,Nf+1) = 1;
Amatrix = Amatrix+Amatrix';

% original pdf
L0 = TheJournal.Eif.Lambda;
e0 = TheJournal.Eif.eta;
m0 = L0 \ e0;

% reorder based upon "feature" adjacency matrix
ii = symamd(Amatrix);
kk = zeros(Naug,1);
for cc=1:TheJournal.Index.Nv
  kk(cc:Nv:Naug) = ii;
end
kk = Nv*(kk-1) + repmat([1:12]',Nf+1,1);
L1 = TheJournal.Eif.Lambda(kk,kk);
e1 = TheJournal.Eif.eta(kk);
m1 = TheJournal.Eif.mu(kk);

% reorder based upon state adjacency matrix
jj = symamd(TheJournal.Eif.Lambda);
L2 = TheJournal.Eif.Lambda(jj,jj);
e2 = TheJournal.Eif.eta(jj);
m2 = TheJournal.Eif.mu(jj);

