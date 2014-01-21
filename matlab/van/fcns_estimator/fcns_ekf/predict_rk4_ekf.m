function [Xaug,Paug,Fv] = predict_rk4_ekf(PMfhandle,Xaug,Paug,index_t,Uv,Qv,dt)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-21-2003      rme         Created & written.
%    08-09-2003      rme         Updated to work with augmented
%                                state vector.
%    04-10-2004      rme         added TheJournal.Index arg to feval
%    04-12-2004      rme         Changed input arguments to Xaug,Paug
%                                instead of using TheJournal directly
%    04-21-2004      rme         Updated RK4 to be consistent with
%                                Hildebrand notation
%    09-09-2004      rme         Renamed from ekfpredict.m to predict_rk4_ekf.m
  
% shorthand index into augmented state vector
Xv_i = index_t.Xv_i; % vehicle state index
Xv   = Xaug(Xv_i);   % vehicle state vector
Xf_i = index_t.Xf_i; % feature state index (all)

if isempty(Uv)
  Uv = 0;
end

%==================================
% STATE PREDICTION
%================================== 
% evaluate the CT process model to get Xv_dot
if nargout(func2str(PMfhandle)) == 2
  % process model function returns jacobian
  [Xv_dot,Fv] = feval(PMfhandle,Xv,Uv,index_t);
else
  [Xv_dot] = feval(PMfhandle,Xv,Uv,index_t);
  % evalate the Jacobian of the process model at the current estimate
  % of the system state
  Fv = numerical_jacobian(PMfhandle,Xv,Xv_dot,[],Uv,index_t);
  Fv = sparse(Fv);
end

% solve differential state equation using 4th order Runge-Kutta to get
% predicted state
k1 = dt*Xv_dot;
k2 = dt*feval(PMfhandle,Xv+k1/2,Uv,index_t);
k3 = dt*feval(PMfhandle,Xv+k2/2,Uv,index_t);
k4 = dt*feval(PMfhandle,Xv+k3  ,Uv,index_t);
% note that only the vehicle state evolves through process model,
% feature states are stationary
Xaug(Xv_i) = Xv + 1/6*(k1 + 2*k2 + 2*k3 + k4);


% solve differential covariance equation using 4th order Runge-Kutta to
% get predicted covariance
k1 = dt*pdot(Fv,Paug(Xv_i,:)     ,Qv,Xv_i,Xf_i);
k2 = dt*pdot(Fv,Paug(Xv_i,:)+k1/2,Qv,Xv_i,Xf_i);
k3 = dt*pdot(Fv,Paug(Xv_i,:)+k2/2,Qv,Xv_i,Xf_i);
k4 = dt*pdot(Fv,Paug(Xv_i,:)+k3  ,Qv,Xv_i,Xf_i);  
Paug(Xv_i,:) =  Paug(Xv_i,:) + 1/6*(k1 + 2*k2 + 2*k3 + k4);
if ~isempty(Xf_i)
  Paug(Xf_i,Xv_i) =  Paug(Xv_i,Xf_i)';
end


%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function Pva_dot = pdot(Fv,Pva,Qv,Xv_i,Xf_i);

Nv = length(Xv_i);    % number of vehicle states
Nf = size(Pva,2)-Nv;  % number of feature states, Naug = (Nv + Nf)
Pv = Pva(Xv_i,Xv_i);  % vehicle state covariance


Pva_dot = zeros([Nv,Nv+Nf]);
Pva_dot(Xv_i,Xv_i) = Fv*Pv + Pv*Fv' + Qv; % differential equation for vehicle state covariance
if Nf > 0
  Pva_dot(Xv_i,Xf_i) = Fv*Pva(Xv_i,Xf_i); % differential equation for augmented state covariance
end
