function [Fk,Bk,uk,Qk] = pwcContinuousToDiscrete(PMfhandle,mu_v,index_t,Uv,Qv,dt)
%function [Fk,Bk,uk,Qk] = pwcContinuousToDiscrete(PMfhandle,mu_v,index_t,Uv,Qv,dt)  
%
% Approximate the nonlinear continuous-time process model to 1st order
% using a Taylor series representation and map to a discrete time model
% assuming piecewise-constant over the timer period delta t.
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-15-2004      rme         Created from ekfpredict_pwc.m
%    09-01-2004      rme         changed Xv to mu_v
%    10-28-2004      rme         Major code reorganization.
%    03-02-2005      rme         Found an analytical error in the calculation of Bk
%                                Turns out using pinv(Fv) was wrong.  However, it 
%                                to a closed-form solution that, though wrong, worked
%                                because of the zeros in uk.  I swithed to numerically
%                                compute Bk using Runge-Kutta
%    03-23-2005      rme         Had bug in Rung-Kutta for Bk, switch to Simpson's rule

%==================================
% STATE PREDICTION
%================================== 
% evaluate the CT process model to get E[Xv_dot]
if nargout(func2str(PMfhandle)) == 2
  % process model function returns jacobian
  [Xv_dot,Fv] = feval(PMfhandle,mu_v,Uv,index_t);
else
  Xv_dot = feval(PMfhandle,mu_v,Uv,index_t);
  % evalate the Jacobian of the process model at the current estimate
  % of the system state
  Fv = numerical_jacobian(PMfhandle,mu_v,Xv_dot,[],Uv,index_t);
end

% The nonlinear differential equation d(x_v)/dt = f(x_v) + w(t)
% is approximated to first order by the piecewise constant linear
% differential equation
%
% d(x_v(t))/dt ~= f(mu_v) + Fv*(x_v(t) - mu_v) + w(t)
% regrouping terms we get
% d(x_v(t))/dt ~= Fv*x_v(t) + {f(mu_v) - Fv*mu_v)} + w(t)
% 
% where Fv is the Jacobian of the nonlinear model evaluated at the
% current estimate of the mean mu_v.
%
% The above can be translated into a discrete linear difference equation
% of the form
%
% x[k+1] = F[k]*x[k] + B[k]*u[k] + w[k]
%
% where F[k] is the state transition matrix for the time interval 
% tk <= t < tk+1, B[k] is the discrete input gain matrix, 
% u[k] = u(tk) = {f(mu_v) - Fv*mu_v}, and w[k] is discrete white noise.

% use van Loan method to compute state transition matrix and associated
% process noise covariance, see Brown & Hwang reference
Nv = length(mu_v);
Fv = full(Fv);
Qv = full(Qv);
A = [-Fv,       Qv; ...
     zeros(Nv), Fv'];
B = expm(A*dt);
Fk = B(Nv+1:end,Nv+1:end)'; % discrete state transition matrix
Qk = Fk * B(1:Nv,Nv+1:end); % discrete process covariance

% compute input and its gain matrix
uk = Xv_dot - Fv*mu_v;
%Bk = (Fk - eye(Nv))*pinv(Fv); % old closed form soln, gives correct answer
%                              % for upper-left 6x6 which is the only part that
%                              % matters since uk(7:12) is zero.  though it gives
%                              % the correct answer, i haven't been able to mathematically
%                              % prove why the pseudo-inverse should work.  i had
%                              % a hunch that it might, so i tried it to verify, but
%                              % will leave it commented until i can prove why it works...
%
% compute gain matrix by numerical integration using Simpson's Rule
% Bk = exp(Fv*t_{t+1}) \int_{tk}^{t_{k+1}} exp(-Fv*tau)dtau
Bk = expm(Fv*dt)*defint(@Bdot,0,dt,6,'simpson',Fv);


%===================================================================
function dBdt = Bdot(t,Fv)
dBdt = expm(-Fv*t);
