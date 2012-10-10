A = symbmatmake('A',4,4,'real');
A = triu(A)+triu(A,1)'

x = symbmatmake('x',4,1,'real');

n = symbmatmake('n',4,1,'real');

alpha = exp(-0.5*x(1)*A(1,1)*x(1) + n(1)*x(1));

p_x1_x2_x3_x4 = exp(-0.5*x'*A*x + n'*x)

p_x1_x2_x3_x4 = collect(p_x1_x2_x3_x4,alpha)

p_x1_x2_x3_x4_sparse = subs(p_x1_x2_x3_x4,{A(1,2),A(1,4)},{0,0})

p_x1_sparse = int(int(int(p_x1_x2_x3_x4_sparse,x(4)),x(3)),x(2))
