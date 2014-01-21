function [PtsVan,PtsTwo] = points2vec(Points,TheJournal,Hwl,list);
%function [PtsVan,PtsTwo] = points2vec(Points,TheJournal,Hwl,list);  

% pre-allocate
N = 5e5;
Tmp.pair   = zeros(2,N);
Tmp.Xci    = zeros(3,N);
Tmp.Xl     = zeros(3,N);
Tmp.Xw     = [];
Tmp.alphaN = zeros(1,N);
Tmp.betaN  = zeros(1,N);
Tmp.gammaN = zeros(1,N);
Tmp.alphaS = zeros(1,N);
Tmp.betaS  = zeros(1,N);
Tmp.gammaS = zeros(1,N);

fields = {'Xci','Xl','alphaN','betaN','gammaN','alphaS','betaS','gammaS'};;

PtsVan = Tmp;
PtsTwo = Tmp;

% find all camera pairs
[Npr,Npc] = size(Points);
[Fni,Fnj] = find(TheJournal.Links.vlinks==1);

% construct pairwise point cloud
ii = 1;
Npairs = length(Fni);
for k=1:Npairs;
  if mod(k,10)==0;
    fprintf('%d of %d\n',k,Npairs);
  end;
  
  % feature number
  fni = Fni(k);
  fnj = Fnj(k);
  
  if fni > Npr || fnj > Npc || isempty(Points(fni,fnj).bmag);
    fprintf('skipping...\n');
    continue;
  end;
  if exist('list','var') && ~isempty(list) && ismember([fni fnj],list,'rows');
    fprintf('in ignore list...\n');
    continue;
  end;

  % stuff data structures
  jj = ii + length(Points(fni,fnj).Van.gammaN) - 1;
  PtsVan.pair(1,ii:jj) = fni;
  PtsVan.pair(2,ii:jj) = fnj;
  PtsTwo.pair(1,ii:jj) = fni;
  PtsTwo.pair(2,ii:jj) = fnj;
  for k=1:length(fields);
    PtsVan.(fields{k})(:,ii:jj) = Points(fni,fnj).Van.(fields{k});
    PtsTwo.(fields{k})(:,ii:jj) = Points(fni,fnj).Twoview.(fields{k});
  end;
  ii = jj+1;
end;

% discard any extra elements
fields = {fields{:},'pair'};
for k=1:length(fields);
  PtsVan.(fields{k}) = PtsVan.(fields{k})(:,1:jj);
  PtsTwo.(fields{k}) = PtsTwo.(fields{k})(:,1:jj);
end;

% transform from local-level to world frame
PtsVan.Xw = Hwl * [PtsVan.Xl; ones(1,size(PtsVan.Xl,2))];
PtsTwo.Xw = Hwl * [PtsTwo.Xl; ones(1,size(PtsTwo.Xl,2))];
