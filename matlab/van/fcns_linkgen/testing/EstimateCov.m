% EstimateCov.m

function [Sigma,Pest] = EstimateCov(n);

Sigma = randcov(n);
Lambda = Sigma\eye(n);


Icol = [zeros(n-1,1); eye(1)];

Pest = rand(n,1);


flag = 0;

B = Icol;
R_old = B - Lambda*Pest;

for i=1:100000

    Z_old = R_old;
    rho = R_old'*Z_old;

    if(i == 1)
        P = Z_old;
    else
        beta = rho/rho_old;
        P = Z_old + beta*P_old;
    end;
    
    Q = Lambda*P;
    alpha = rho/(P'*Q);
    
    Pest = Pest + alpha*P;

    
    R_old = R_old - alpha*Q;
    P_old = P;
    rho_old = rho;
    
    Error = B - Lambda*Pest;
    
    if(sqrt(Error'*Error) < 0.01)
        fprintf(1,'Achieved L2(Error) = %.4f \n',sqrt(Error'*Error));
        return;
    end;
    
end;



return;