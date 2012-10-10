function [X_traj,t_traj,m_traj,d_traj] = simtraj(t,X,goal_t,samp_period)
%SIMTRAJ generate and sample a smooth vehicle trajectory behavior
%   [X_TRAJ,T_TRAJ,M_TRAJ,D_TRAJ] = SIMTRAJ(T,X,GOAL_TYPE,PARAM_T,SAMP_PERIOD)
%
%   INPUTS:
%   T: current mission time in seconds
%   X: [12 x 1] vector of the current vehicle state where the
%      vehicle state vector is assumed to contain the following elements
%      X = [x y z r p h vx vy vz rr pr hr]'
%           1 2 3 4 5 6 7  8  9  10 11 12   index
%   GOAL_TYPE: vehicle behavior type as defined by function MODE_TYPE_STRUCT
%              XY_GOTO, XY_LINE, XY_CIRCLE, XY_SQUIGGLE
%   PARAM_T:  parameter structure associated with behavior
%   SAMP_PERIOD: [N x 1] vector of sensor sampling periods
%
%   OUTPUTS:
%   X_TRAJ: [12 x M] vector of sampled vehicle states
%   T_TRAJ: [ 1 x M] vector of corresponding sampling times
%   M_TRAJ: [ 1 x M] vector of sensor sampling index
%   D_TRAJ: [ 1 x M] vector of desired vehicle heading trajectory (OPTIONAL)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-20-2003      rme         Created and written.
%    12-19-2003      rme         Modified pitch and roll rates to be out
%                                of phase

clear unwrap_heading;
mode_type_t = mode_type_struct;

goal_type = goal_t.XYMode;
param_t = goal_t.param;

TWOPI = 2*pi;
MAX_HEADING_RATE = 10*DTOR; % [rad/s]
[EULER,RUNGEKUTTA] = enumerate;
int_method = EULER;

% define shorthand index for state vector elements
% vehicle state vector is assumed to contain the following elements
% X = [x y z r p h vx vy vz rr pr hr]'
%      1 2 3 4 5 6 7  8  9  10 11 12   index
xj  =  1; yj  =  2; zj  =  3;
rj  =  4; pj  =  5; hj  =  6;
vxj =  7; vyj =  8; vzj =  9;
rrj = 10; prj = 11; hrj = 12;

% initial conditions
desired_heading = NaN;
dt = 0;
t_o  = t;
X_o = X;
speed_o = sqrt(X_o(vxj)^2 + X_o(vyj)^2);
vz_o = X_o(vzj);
rr_o = X_o(rrj);
pr_o = X_o(prj);

% add some jitter to sensor suite sample periods
% to make them asyncronous
samp_time = t_o + 0.1*rand(size(samp_period));

X_traj = zeros(12,1e4);
t_traj = zeros(1,1e4);
m_traj = zeros(1,1e4);
d_traj = zeros(1,1e4);
done = false;
kount = 0;
while ~done
  kount = kount+1;
  
  %=============================
  % SELECT XY BEHAVIOR
  %=============================
  switch goal_type
    
   case mode_type_t.XY_GOTO      
    % generate a heading rate to go towards desired way point
    desired_heading = atan2(param_t.stop(2)-X(yj), param_t.stop(1)-X(xj));
    % terminate condition
    distanceToGoal = sqrt((param_t.stop(1)-X(xj))^2 + (param_t.stop(2)-X(yj))^2);
    if distanceToGoal < 0.25
      done = true;
    end
    
   case mode_type_t.XY_LINE
    LINE_BAND = 10; % [m]
    % generate a trajectory to get vehicle on a line
    p_a = param_t.start';  % start pos
    p_b = param_t.stop';   % stop  pos
    p_c = X([xj,yj]);      % current pos
    v_ab = p_b-p_a;        % vector from a to b
    v_ac = p_c-p_a;        % vector from a to c
    v_bc = p_c-p_b;        % vector from b to c
    u_ab = unitize(v_ab);  % unit vector from a to b
    u_ac = unitize(v_ac);  % unit vector from a to c
    u_bc = unitize(v_bc);  % unit vector from b to c
    s = v_ac'*u_ab;        % projection along a to b
    p_int = p_a + s*u_ab;  % vehicle orthog intersection point with line ab
    if s < 0 % before line segment
      % go to start point
      p_goal = p_a;
    elseif (p_int-p_b)'*(-u_ab) < 0 % past line segment
      % go to end point
      p_goal = p_b;
    else % get on line
      d_orthog =sqrt(v_ac'*v_ac-s^2); % orthog distance
      if d_orthog > LINE_BAND
	% head straight for intersection point
	p_goal = p_int;
      else
	% set goal along line A->B
	fudge = param_t.speed/0.25;
	p_goal = p_int + max(fudge*(u_ac'*u_ab),0.1)*u_ab;
      end
    end
    desired_heading = atan2(p_goal(2)-p_c(2), p_goal(1)-p_c(1));    
    % terminate condition
    distanceToGoal = sqrt((param_t.stop(1)-X(xj))^2 + (param_t.stop(2)-X(yj))^2);
    if distanceToGoal < 0.25
      done = true;
    end
    
   case mode_type_t.XY_CIRCLE
    hr = param_t.speed/param_t.radius;
    X(hrj) = hr;
    if ~exist('complete','var'); complete = 0; end
    complete = complete + hr*dt;
    % terminating condition
    if complete > param_t.angle
      done = true;
    end

   case mode_type_t.XY_SQUIGGLE
    if ~exist('hgoalmet','var'); hgoalmet = false; end
    if hgoalmet == false
      % get on heading goal
      desired_heading = param_t.heading;
      delta_h = abs(mod(X(hj)+2*pi,2*pi) - mod(desired_heading+2*pi,2*pi));
      if delta_h < 0.1*DTOR
	hgoalmet = true;
	desired_heading = NaN;
      end
    else
      % squiggle
      X(hrj) = param_t.heading_rate*sin(2*pi*t/param_t.heading_period);
      % remove integeration drift effects
      X(hj) = param_t.heading + ...
	      -param_t.heading_rate*param_t.heading_period/(2*pi)*cos(2*pi*t/param_t.heading_period);
    end
    % terminate condition
    if sqrt((X(xj)-X_o(xj))^2+(X(yj)-X_o(yj))^2) > param_t.distance
      done = true;
    end
      
   otherwise
    disp('Unknown Method');
    done = true;
  end % switch goal_t.XYMode

  % heading controller
  if ~isnan(desired_heading)
    % unwrap desired heading 
    desired_heading = unwrap_heading(desired_heading,1);
    if kount == 1
      X(hj) = unwrap_heading(X(hj),0);
    end
    % adjust vehicle heading rate based upon heading error
    delta_h = desired_heading-X(hj);
    alpha = 0.90;
    gain  = 1;
    X(hrj) = alpha*X(hrj)+(1-alpha)*sign(delta_h)*min(gain*abs(delta_h),MAX_HEADING_RATE);
    X(hrj) = sign(X(hrj))*min(abs(X(hrj)),MAX_HEADING_RATE);  
  end
    
  % decoupled state elements with smooth transition
  alpha  = exp(-(t-t_o)/3);  
  speed  = alpha*speed_o + (1-alpha)*param_t.speed;
  X(vxj) = speed*cos(X(hj));
  X(vyj) = speed*sin(X(hj));
  X(vzj) = alpha*X(vzj) + (1-alpha)*param_t.ascent_rate*cos(2*pi*t/param_t.ascent_period);
  X(rrj) = alpha*X(rrj) + (1-alpha)*param_t.roll_rate*-cos(2*pi*t/param_t.roll_period); %out of phase from pitch
  X(prj) = alpha*X(prj) + (1-alpha)*param_t.pitch_rate*cos(2*pi*t/param_t.pitch_period);
  

  % update state vector
  [dt,samp_time,mindx] = calc_dt(t,samp_time,samp_period);  
  t = t + dt;
  if int_method == EULER
    X([xj,yj,zj]) = X([xj,yj,zj]) + X([vxj,vyj,vzj])*dt;
    X([rj,pj,hj]) = X([rj,pj,hj]) + X([rrj,prj,hrj])*dt;
  elseif int_method == RUNGEKUTTA
    X = rungekutta(@plant_model,[],X,dt);
  end
  
  % store output
  X_traj(:,kount) = X;
  t_traj(kount) = t;
  m_traj(kount) = mindx;
  d_traj(kount) = desired_heading;
end % while ~done

% remove zero elements
X_traj = X_traj(:,1:kount);
t_traj = t_traj(1:kount);
m_traj = m_traj(1:kount);
d_traj = d_traj(1:kount);

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function [dt,samp_time,ind] = calc_dt(t,samp_time,samp_period)
dt = samp_time-t;
[dt,ind] = min(dt);
samp_time(ind) = samp_time(ind)+samp_period(ind);


%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%function h = unwrap_heading0(h,flag)
%persistent h_old;
%if isempty(h_old); h_old = h; end
%tmp = unwrap([h_old,h]);
%h = tmp(2);
%h_old = h;

%function h = unwrap_heading1(h,flag)
%persistent h_old;
%if isempty(h_old); h_old = h; end
%kount = 1;
%while abs(h-h_old) > pi && kount<10
%  h = h + sign(h_old)*2*pi;
%  kount = kount+1;
%end
%if flag > 0
%  h_old = h;
%end

function h = unwrap_heading(h,flag)
persistent h_old;
if isempty(h_old); h_old = h; end
TWOPI = 2*pi;
while abs(h-h_old) > pi
  hplus = h+TWOPI;
  if abs(hplus-h_old) > abs(h-h_old)
    h = h-TWOPI;
  else
    h = hplus;
  end
end
if flag > 0
  h_old = h;
end


%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
function X_dot = plant_model(X)
% define shorthand index for state vector elements
% vehicle state vector is assumed to contain the following elements
% X = [x y z r p h vx vy vz rr pr hr]'
%      1 2 3 4 5 6 7  8  9  10 11 12   index
xj  =  1; yj  =  2; zj  =  3;
rj  =  4; pj  =  5; hj  =  6;
vxj =  7; vyj =  8; vzj =  9;
rrj = 10; prj = 11; hrj = 12;

X_dot = zeros(size(X));
X_dot([xj,yj,zj]) = X([vxj,vyj,vzj]);
X_dot([rj,pj,hj]) = X([rrj,prj,hrj]);
