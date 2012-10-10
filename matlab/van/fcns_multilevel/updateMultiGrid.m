function updateMultiGrid(level,Fh);

global TheJournal MG;

if level == 1;
  % shorthand index
  Nf    = TheJournal.Eif.Nf;
  Nv    = TheJournal.Eif.Nv;
  Xp_i  = TheJournal.Eif.Xp_i;
  Xa_i  = TheJournal.Eif.Lambda;
  Xf_ii = TheJournal.Eif.Xf_ii;

  % update entries in finest level representation
  ii = [Xf_ii{Fh}];
  MG(level).Ah(ii,ii) = TheJournal.Eif.Lambda(ii,ii);
  MG(level).bh(ii)    = TheJournal.Eif.eta(ii);
  MG(level).eh(ii)    = TheJournal.Eif.mu(ii);
end;
  
updateInterpolator(level,Fh);


%==================================================================
function FH = updateInterpolator(level,Fh);

global TheJournal MG;

for ii=1:length(Fh);
  fh = Fh(ii);                         % fine feature number
  fH = coarseFeature(Fh,MG.Nh{level}); % coarse feature number(s)
  if length(fH) == 1;
    MG(level).IhH(fh,fH) 
  else;
    
  end;
end;
  

%===================================================================
function fH = coarseFeature(level,fh,Nh);

global MG;

% note: Matlab indexing begins with 1, therefore first subtract 1 to start indexing at 0
fh = fh-1;

% compute coarse feature number(s) associated with fine feature fh
if mod(fh,2) == 0;         % even
  fH = fh/2;
elseif fh == Nh;           % odd & last
  fH = (fh+1)/2;
else;
  fH = [(f-1)/2,(f+1)/2];  % otherwise
end;

% add 1 to put back into Matlab indexing
fH = fH+1;
