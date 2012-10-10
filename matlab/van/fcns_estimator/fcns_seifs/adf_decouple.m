function [etaADF,LambdaADF] = adf_decouple(etaADF,LambdaADF,fni,fnj,index_t)
%function [etaADF,LambdaADF] = adf_decouple(etaADF,LambdaADF,fni,fnj,index_t)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-22-2004      rme         Created and written.

% project LambdaADF to a decoupled constrained PDF
fprintf('\nSPARSIFYING (%d,%d)...\n\n',fnj,fni);
if fnj == index_t.Nf
  xt = [index_t.Xf_ii{fnj},index_t.Xv_i];
  mminus = [index_t.Xf_ii{[1:fni-1,fni+1:fnj-1,fnj+1:index_t.Nf]}];
else
  xt = [index_t.Xf_ii{fnj}];
  mminus = [index_t.Xf_ii{[1:fni-1,fni+1:fnj-1,fnj+1:index_t.Nf]},index_t.Xv_i];
end
m0     = index_t.Xf_ii{fni};
mplus  = [];

figure(2); spy(LambdaADF); label_corrcoef(index_t);
%set(gca,'grid','-.'); grid on; 
title('LambdaADF prior');
[etaADF,LambdaADF] = sparsify_ryan(etaADF, LambdaADF, xt, m0, mplus, mminus);
figure(3); spy(LambdaADF); label_corrcoef(index_t); 
%set(gca,'grid','-.'); grid on;
title('LambdaADF posterior');
drawnow;
pause(0.75);
