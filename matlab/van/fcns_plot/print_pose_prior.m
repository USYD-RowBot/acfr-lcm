function print_pose_prior(X,P,label,headerflag,footerflag)
%function print_pose_prior(X,P,label,headerflag,footerflag)
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    09-29-2003      rme         Created and written.
%    04-09-2004      rme         Removed some extra args and cleaned up code.
%    04-12-2004      rme         Changed args to not use TheJournal directly.
%    04-13-2004      rme         Renamed from print_prior_pose.m to print_pose_prior.m
  
if headerflag
  fprintf('   \t  x\t  y\t  z\t  r\t  p\t  h\n');
end

printpose(label,X,P);

if footerflag
  fprintf('---------------------------------------------------------------------\n');  
end

%==============================================================================================
function printpose(str,X_lv,Cov_X_lv)
X_lv(4:6) = rem(X_lv(4:6)*RTOD,360);
X_sigma = sqrt(diag(Cov_X_lv));
X_sigma(4:6) = X_sigma(4:6)*RTOD;

fprintf('X%s:',str); fprintf('\t%+0.2f',X_lv); fprintf('\n');
fprintf('+/-'); fprintf('\t %0.2f',X_sigma); fprintf('\t trace=%.2e',X_sigma'*X_sigma); fprintf('\n');
