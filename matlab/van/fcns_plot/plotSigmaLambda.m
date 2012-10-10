function plotSigmaLambda(TheJournal,TheConfig)
%function plotSigmaLambda(TheJournal,TheConfig)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-18-2004      rme         Created and written.
%    12-20-2004      rme         Added Naug.

Naug = TheJournal.Index.Naug;
if TheConfig.Estimator.useEkf && TheConfig.Estimator.useEif;
  subplot(1,2,1);
  plot_corrcoef(TheJournal.Ekf.Sigma(1:Naug,1:Naug),1);
  label_corrcoef(TheJournal.Index,0);
  title('Sigma Normalized');
  subplot(1,2,2);
  plot_corrcoef(TheJournal.Eif.Lambda(1:Naug,1:Naug),1);
  label_corrcoef(TheJournal.Index,0);
  title('Lambda Normalized');
elseif TheConfig.Estimator.useEkf;
  plot_corrcoef(TheJournal.Ekf.Sigma(1:Naug,1:Naug),1);
  label_corrcoef(TheJournal.Index,0);
  title('Sigma Normalized');  
elseif TheConfig.Estimator.useEif;
  plot_corrcoef(TheJournal.Eif.Lambda(1:Naug,1:Naug),1);
  label_corrcoef(TheJournal.Index,0);
  title('Lambda Normalized');
else;
  error('no estimator selected in TheConfig');
end;

