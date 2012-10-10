function label_corrcoef(index_t,labelflag,fignum)
%LABEL_CORRCOEF labels correlation coefficient matrix.
%function plot_corrcoef(index_t,labelflag,fignum)  
%  index_t: state index structure
%  labelflag: (opt) true plots feature labels instead of feature numbers  
%  fignum:  (opt) figure number, defaults to current figure
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    03-30-2004      rme         Created from plot_corrcoef.m
%    04-14-2004      rme         Reordered inputs.
%    10-14-2004      rme         Modified to work with reordered state vector.
  
if ~exist('labelflag','var') || isempty(labelflag), labelflag=false; end
if ~exist('fignum','var') || isempty(fignum), fignum = gcf; end

figure(fignum);
for ii=1:index_t.Nf
  if labelflag
    label{ii,1} = sprintf('%d',index_t.featureLUT(ii));
  else
    label{ii,1} = num2str(ii);
  end
  tick(ii,1) = index_t.Xf_ii{ii}(1);
end
label{ii+1} = 'Xv';
tick(ii+1) = index_t.Xv_i(1);
set(gca,'YTick',tick,'XTick',tick);
set(gca,'YTickLabel',label,'XTickLabel',label);
set(gca,'YAxisLocation','left','XAxisLocation','bottom');
%grid(gca,'on');
