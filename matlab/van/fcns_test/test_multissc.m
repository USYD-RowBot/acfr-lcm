clear all;

N = 1e0;

x_ij = rand(6,N);
x_jk = rand(6,N);


% ssc_inverse: x_ji = (-)x_ij
tic;
xn_ji = zeros(size(x_ij));
Jn_ji = zeros(6,6,N);
for n=1:N;
  [xn_ji(:,n),Jn_ji(:,:,n)] = inverse_orig(x_ij(:,n));
end;
tn_ji = toc;
 
tic;
[xm_ji,Jm_ji] = ssc_inverse(x_ij);
tm_ji = toc;

fprintf('ssc_inverse: %g %g %g %g %g\n', ...
        any(xm_ji(:)-xn_ji(:)), ...
        any(Jm_ji(:)-Jn_ji(:)), ...
        tm_ji, tn_ji, tn_ji/tm_ji);


% ssc_head2tail: x_ik = x_ij (+) x_jk
tic;
xn_ik = zeros(size(x_ij));
Jn_ik = zeros(6,12,N);
for n=1:N;
  [xn_ik(:,n),Jn_ik(:,:,n)] = head2tail_orig(x_ij(:,n),x_jk(:,n));
end;
tn_ik = toc;

tic;
[xm_ik,Jm_ik] = ssc_head2tail(x_ij,x_jk);
tm_ik = toc;

fprintf('ssc_head2tail: %g %g %g %g %g\n', ...
        any(xm_ik(:)-xn_ik(:)), ...
        any(Jm_ik(:)-Jn_ik(:)), ...
        tm_ik, tn_ik, tn_ik/tm_ik);


% ssc_tail2tail: x_jk = (-)x_ij (+) x_ik
tic;
xn_jk = zeros(size(x_ij));
Jn_jk = zeros(6,12,N);
for n=1:N;
  [xn_jk(:,n),Jn_jk(:,:,n)] = tail2tail_orig(x_ij(:,n),xm_ik(:,n));
end;
tn_jk = toc;
 
tic;
[xm_jk,Jm_jk] = ssc_tail2tail(x_ij,xm_ik);
tm_jk = toc;

fprintf('ssc_tail2tail: %g %g %g %g %g\n', ...
        any(xm_jk(:)-xn_jk(:)), ...
        any(Jm_jk(:)-Jn_jk(:)), ...
        tm_jk, tn_jk, tn_jk/tm_jk);

fprintf('\n');
