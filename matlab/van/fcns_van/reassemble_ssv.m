function ssv_t = reassemble_ssv(ssvdir);
%function ssv_t = reassemble_ssv(ssvdir);
%  
% History
% Date          Who        Comment
% ----------    ---        -----------------------------------
% 2006-01-09    rme        Created and written.
% 2006-01-12    rme        Return empty ssv_t if K = 0.

if ssvdir(end) ~= '/';
  ssvdir(end+1) = '/';
end;

d_t = dir([ssvdir,'ssv-*.mat.gz']);

K = length(d_t);
if K == 0;
  ssv_t = [];
  return;
end;

for ii=1:K
  cmd = sprintf('!env gunzip -c %s%s > /tmp/ssv_t.mat',ssvdir,d_t(ii).name);
  eval(cmd);
  out = load('/tmp/ssv_t');
  fprintf('Assembling ssv_t: %s   [%d of %d]\r',d_t(ii).name,ii,length(d_t));
  
  % pre-allocate
  if ii==1;
    N = length(out.ssv_t.Index.Xv_i);
    M = 30000;
    ssv_t.t = zeros(1,M);
    ssv_t.mu_x = zeros(N,M);
    ssv_t.Sigma_xx = zeros(N,N,M);
    ssv_t.camflag = zeros(1,M);
    ssv_t.cc = 0;
    ssv_t.Index = out.ssv_t.Index;
  end;
  
  % concatenate
  kk = [1:length(out.ssv_t.t)]+ssv_t.cc;
  ssv_t.t(kk)            = out.ssv_t.t;
  ssv_t.mu_x(:,kk)       = out.ssv_t.mu_x;
  ssv_t.Sigma_xx(:,:,kk) = out.ssv_t.Sigma_xx;
  ssv_t.camflag(kk)      = out.ssv_t.camflag;
  ssv_t.cc               = kk(end);
  
  % trim excess
  if ii==K;
    kk = 1:ssv_t.cc;
    ssv_t.t = ssv_t.t(kk);
    ssv_t.mu_x     = ssv_t.mu_x(:,kk);
    ssv_t.Sigma_xx = ssv_t.Sigma_xx(:,:,kk);
    ssv_t.camflag  = ssv_t.camflag(kk);
  end;
end;
fprintf('\n');
