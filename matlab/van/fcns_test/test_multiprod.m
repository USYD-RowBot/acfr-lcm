clear all;

N = 100000;
[r,p,h] = deal(rand(N,1),rand(N,1),rand(N,1));

tic;
Rm = rotxyz(r,p,h);
Im = multiprod(Rm,multitransp(Rm));
tm = toc;
fprintf('Multiprod = %g\n',tm);

tic;
Rn = zeros(size(Rm));
In = zeros(size(Im));
for n=1:length(r);
  R = rotxyz([r(n),p(n),h(n)]);
  I = R*R';
  Rn(:,:,n) = R;
  In(:,:,n) = I;
end;
tn = toc;
fprintf('for loop  = %g\n',tn);
fprintf('Speed up  = %g\n',tn/tm);
