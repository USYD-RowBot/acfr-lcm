function [y_est] = motion_center_homo(y_o,u1,v1,u2,v2)

% vector representation
x1 = [u1'; v1'];
x2 = [u2'; v2'];

% construct motion vector
m = (x2-x1);

% construct an orthogonal vector
m_perp = m([2,1],:);
m_perp(1,:) = -m_perp(1,:);

% compute midpoint
p1 = 0.5*(x2 + x1);

% compute a point which lies on vector orthogonal to motion vector
p2 = p1 + m_perp;

% homogenous representation
P1 = homogenize(p1(1,:)',p1(2,:)');
P2 = homogenize(p2(1,:)',p2(2,:)');

% equation of line that contains these two points & which is
% orthogonal to motion vector
L = cross(P1,P2);
mag = sqrt(dot(L,L));
L = L./repmat(mag,[3 1]);

y_o = y_o/norm(y_o);

% compose motion direction vectors
mvec = x2 - x1;
mag = dot(mvec,mvec);
mvec = mvec./repmat(sqrt(mag),[2 1]);

% define optimization settings
options = optimset('Diagnostics','off', ...       % print diagnostic info
		   'LevenbergMarquardt','on', ... % choose LM over Gauss-Newton
		   'LargeScale','off', ...
		   'MaxFunEvals',1e6, ...         % let it iterate indefinitely
		   'MaxIter',50, ...              % if initialized correctly, should coverge very quickly
		   'Display','off');             % level of display

% perform nonlinear minimization
[y_est,fval,exitflag] = ...
    fmincon(@dotproduct_error,y_o,[],[],[],[],[],[],@norm_constraint,options,L);


%y_est = y_est/y_est(3);

%******************************************************************************
function cost = dotproduct_error(y,L)

y = y/sqrt(y'*y);  

cost_vector = 90 - acos(L'*y)*RTOD;
cost = cost_vector'*cost_vector;

%******************************************************************************
function [C,Ceq] = norm_constraint(y,varargin)

% Non-linear inequality constraints
C = [];

% Non-linear equality constraints
Ceq = 1-sqrt(y'*y);
