function [etaADF,LambdaADF] = adf_markov(etaADF,LambdaADF,fni,fnj,index_t)
%function [etaADF,LambdaADF] = adf_markov(etaADF,LambdaADF,fni,fnj,index_t)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-22-2004      rme         Created and written.

% project LambdaADF to a Markov constrained PDF
if fnj-fni > 1
  for fnj=fnj:-1:fni+2
    fprintf('\nSPARSIFYING (%d,%d)...\n\n',fnj,fni);
    xt     = [index_t.Xf_ii{fnj:end}];
    mplus  = markov_blanket(LambdaADF,xt);
    m0     = setdiff(mplus,index_t.Xf_ii{fnj-1});
    mplus  = setdiff(mplus,m0);
    mminus = [index_t.Xf_i,index_t.Xv_i];
    mminus([xt,m0,mplus]) = [];
    figure(2); spy(LambdaADF); label_corrcoef(index_t);
    %set(gca,'grid','-.'); grid on; 
    title('LambdaADF prior');
    [etaADF,LambdaADF] = sparsify_ryan(etaADF, LambdaADF, xt, m0, mplus, mminus);
    figure(3); spy(LambdaADF); label_corrcoef(index_t); 
    %set(gca,'grid','-.'); grid on;
    title('LambdaADF posterior');
    drawnow;
    pause(0.75);
  end
end  
