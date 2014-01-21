
% load jhu results
if ~exist('TJ','var');
  load('/files1/processed/van/output/jhu04-6_gridsurvey3/archive.20041112b/TJ.mat');
  SigmaUB = zeros(Index.Naug,Index.Naug);
  for fn=1:length(TJ);
    Xf_i = Index.Xf_ii{fn};
    SigmaUB(Xf_i,Xf_i) = TJ(fn).Sigma(Xf_i,Xf_i);
  end;
end;

fni = 20;
fnj = 30;

Lambda     = TJ(end).Lambda;
Sigma_true = TJ(end).Sigma;


Xfi = Index.Xf_ii{fni};
Xfj = Index.Xf_ii{fnj};

mplus_i = markov_blanket(Lambda,[Xfi,Xfj]);
mminus_i = Index.Xa_i;
mminus_i([Xfi,Xfj,mplus_i]) = [];

Lambda_xx   = Lambda([Xfi,Xfj],[Xfi,Xfj]);
Lambda_xMp  = Lambda([Xfi,Xfj],mplus_i);
Lambda_MpMm = Lambda(mplus_i,mminus_i);
Lambda_MpMp = Lambda(mplus_i,mplus_i);
Sigma_MmMm  = SigmaUB(mminus_i,mminus_i);

Lambda_joint = [Lambda_xx,   Lambda_xMp; ...
		Lambda_xMp', Lambda_MpMp];
Sigma_joint = spdinverse(Lambda_joint);

Sigma_xx = Sigma_joint(1:12,1:12);

Sigma_xx_true = Sigma_true([Xfi,Xfj],[Xfi,Xfj]);
