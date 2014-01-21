N = 5;
Lambda  = ssa_t.TheJournal{N}.Lambda;
eta     = ssa_t.TheJournal{N}.eta;
index_t = ssa_t.TheJournal{N}.index;

index_t.Xf_ii{end+1} = index_t.Xv_i;

for ii=2:index_t.Nf
  
  % current state element index
  xii = index_t.Xf_ii{ii};
  
  % active features to remain active
  mplus = [index_t.Xf_ii{[ii-1,ii+1]}];
  
  % any features other than Markov connected
  m0 = setdiff(markov_blanket(Lambda,xii),mplus);
  
  % inactive features
  mminus = [index_t.Xf_i, index_t.Xv_i];
  mminus([xii,m0,mplus]) = [];
  
  % sparsify preserving block tridiagonal structure
  if ~isempty(m0)
    [eta,Lambda] = sparsify_ryan(eta,Lambda,xii,m0,mplus,mminus);
  end
  
  spy(Lambda); drawnow;

end


Sigma = full(Lambda)^-1;

SigmaB = blkTriDiagInv(Lambda,index_t);
