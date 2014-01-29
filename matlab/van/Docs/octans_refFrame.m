x_rs = [ [0; 0; 0] ; [5; -10; 30]*DTOR ]; % octans sensor frame w.r.t. octans ref frame 
					 % (i.e., the raw octans mesasurement)

x_rs_prime    = x_rs;					 
x_rs_prime(6) = -x_rs(6); % negate raw heading, because its reported sign is inconsistent
			  % with the definition of its reference frame

x_lr = [ [0; 0; 0] ; [180; 0; 0]*DTOR ]; % octans ref frame w.r.t. local-level frame

x_vs = [ [0; 0; 0] ; [180; 0; 0]*DTOR ]; % octans sensor frame w.r.t. vehicle frame

% x_lv = x_lr + x_rs + (-x_vs)
x_lv = head2tail(x_lr,head2tail(x_rs_prime,inverse(x_vs)));


fprintf('x_rs = '); fprintf('%+4.1f\t', x_rs(4:6)*RTOD); fprintf('\n');
fprintf('x_lv = '); fprintf('%+4.1f\t', x_lv(4:6)*RTOD); fprintf('\n');
