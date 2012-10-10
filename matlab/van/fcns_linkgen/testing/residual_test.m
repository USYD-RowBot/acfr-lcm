tt = 70;
ii = 1;
jj = 18;

% original system
Lambda1 = ssa_t.TheJournal{tt}.Lambda;
eta1    = ssa_t.TheJournal{tt}.eta;
mu1     = Lambda1 \ eta1;

% add a soft constraint to system
Xf_i = ssa_t.TheJournal{tt}.index.Xf_ii{ii};
Xf_j = ssa_t.TheJournal{tt}.index.Xf_ii{jj};

if false
  Sigma1  = full(Lambda1)^-1;
  Tmp = Sigma1(Xf_j,Xf_j);
  Sigma1(Xf_j,:) = 0;
  Sigma1(:,Xf_j) = 0;
  Sigma1(Xf_j,Xf_j) = 1e4*Tmp;
  Sigma1(Xf_j(1:2),Xf_j(1:2)) = 10e0*eye(2);
  Lambda1 = Sigma1^-1;
  eta1 = Lambda1*mu1;
end
  
% force xi & xj to be coincident
H = spalloc(2,length(eta1),4);
H(:,Xf_i(1:2)) =  speye(2);
H(:,Xf_j(1:2)) = -speye(2);
z = [0; 0];
R  = 10^-3*eye(2);

% constrained system
Lambda2 = Lambda1 + H'*R^-1*H;
eta2    = eta1 + H'*R^-1*z;
mu2     = Lambda2 \ eta2;


% residuals
res1 = eta1 - Lambda1*mu1;
res2 = eta2 - Lambda2*mu2;

figure(1); clf;
plot(mu1(2:12:end),mu1(1:12:end),'b.', ...
     mu2(2:12:end),mu2(1:12:end),'r.');
hold on;
plot(mu1([Xf_i(2),Xf_j(2)]),mu1([Xf_i(1),Xf_j(1)]),'g');
plot(mu2([Xf_i(2),Xf_j(2)]),mu2([Xf_i(1),Xf_j(1)]),'c');
hold off;
axis equal;
grid on;

%C1 = -0.5*mu1'*Lambda1*mu1 + eta1'*mu1;
%C2 = -0.5*mu2'*Lambda1*mu2 + eta1'*mu2;

C3 =  0.5*(mu2-mu1)'*Lambda1*(mu2-mu1)

%disp([C1 C2 C1-C2]);
%disp(log10(C2)-log10(C1));
