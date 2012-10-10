%S{1} = full(TheJournal.Eif.Lambda)^-1;
L{1} = TheJournal.Eif.Lambda;
e{1} = TheJournal.Eif.eta;

ll = 1;
for ii=1:10:TheJournal.Index.Nf+1
  ll = ll+1;
  fprintf('level %d\n',ll);  
  try
    xii{ll} = [TheJournal.Index.Xf_ii{ii:ii+9}];
  catch
    xii{ll} = [TheJournal.Index.Xf_ii{ii:end}];
  end
  mplus{ll} = markov_blanket(L{ll-1},xii{ll});
  mminus{ll} = [TheJournal.Index.Xf_i, TheJournal.Index.Xv_i];
  mminus{ll}([xii{ll},mplus{ll}]) = [];
  
  [e{ll},L{ll}] = sparsify_ryan(e{ll-1},L{ll-1},xii{ll},mplus{ll},[],mminus{ll});

  spy(L{ll}); title(sprintf('Level %d',ll)); drawnow;
end
