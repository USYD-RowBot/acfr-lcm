% construct adjacency matrix
Nf = TheJournal.Index.Nf;
Nv = TheJournal.Index.Nv;
Naug = TheJournal.Index.Naug;
A0 = ( (link_t.vlinks==1) + speye(Nf) + spdiags(repmat([1,0,1],Nf,1),-1:1,Nf,Nf) );
A0(Nf,Nf+1)   = 1;
A0(Nf+1,Nf+1) = 1;
A0 = A0+A0';

S0 = full(TheJournal.Eif.Lambda)^-1;
L0 = TheJournal.Eif.Lambda;
e0 = TheJournal.Eif.eta;

% level 1
xi1 = [TheJournal.Index.Xf_ii{10:32}];
mplus1 = markov_blanket(L0,xii);
mminus1= [TheJournal.Index.Xf_i, TheJournal.Index.Xv_i];
mminus1([xii,mplus]) = [];

[e1,L1] = sparsify_ryan(e0,L0,xii,mplus,[],mminus);

S0a = S0(xii,xii);
S1a = full(L1(xii,xii))^-1;

% level 2
xi2 = [TheJournal.Index.Xf_ii{76:100}, TheJournal.Index.Xv_i];
mplus2 = markov_blanket(L1,xii);
mminus2= [TheJournal.Index.Xf_i, TheJournal.Index.Xv_i];
mminus2([xii,mplus]) = [];

[e2,L2] = sparsify_ryan(e1,L1,xii,mplus,[],mminus);

S0b = S0(xii,xii);
S2b = full(L2(xii,xii))^-1;

% level 2
xi2 = [TheJournal.Index.Xf_ii{76:100}, TheJournal.Index.Xv_i];
mplus2 = markov_blanket(L1,xii);
mminus2= [TheJournal.Index.Xf_i, TheJournal.Index.Xv_i];
mminus2([xii,mplus]) = [];

[e2,L2] = sparsify_ryan(e1,L1,xii,mplus,[],mminus);

S0b = S0(xii,xii);
S2b = full(L2(xii,xii))^-1;
