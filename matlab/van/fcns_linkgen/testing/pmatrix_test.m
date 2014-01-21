Ainv = symmetrize(randcov(4)^-1);
Rinv = symmetrize(randcov(1)^-1);

H12 = sparse([eye(4) -eye(4) zeros(4) zeros(4)]);
H23 = sparse([zeros(4) eye(4) -eye(4) zeros(4)]);
H34 = sparse([zeros(4) zeros(4) eye(4) -eye(4)]);

Lambda = blkdiag(Ainv,Ainv,Ainv,Ainv);
Lambda = Lambda + H12'*Rinv*H12;
Lambda = Lambda + H23'*Rinv*H23;
Lambda = Lambda + H34'*Rinv*H34;


Sigma = Lambda^-1;

P11inv = Lambda(1:8,1:8)^-1;
P22inv = Lambda(9:16,9:16)^-1;
P12 = Lambda(1:8,9:16);
P21 = Lambda(9:16,1:8);
V11 = Lambda(1:8,1:8)^-1;
for ii=1:1
  V22 = P22inv; %+ P22inv*P21*V11*P12*P22inv;
  V11 = P11inv; %+ P11inv*P12*V22*P21*P11inv;
end


V11_true = Sigma(1:8,1:8);
V22_true = Sigma(9:16,9:16);


diff1 = V11_true-V11
diff2 = V22_true-V22

