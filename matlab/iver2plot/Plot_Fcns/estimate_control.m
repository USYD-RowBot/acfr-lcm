function est = estimate_control()

time = evalin('base','iver_t.elaptime');
depth = evalin('base','iver_t.z'); %meters
seafloor = evalin('base','iver_t.seafloor'); %meters
depth_goal = evalin('base','iver_t.ref.Depth_K_Goal'); %meters

%convert to feet *shudder*
est.depth = depth*MTOF;
est.seafloor = seafloor*MTOF;
est.depth_goal = depth_goal*MTOF;

depth_kp = 3.5;
depth_ki = .15;

anglerange   = 25; %degrees
safetyheight = 10; %feet
dt			 = .5;
istatemax	 =  83.3;
istatemin	 = -20;

est.dgoal = min(max(est.depth_goal, est.seafloor+safetyheight),0); %feet
est.depth_err = est.dgoal - est.depth; %feet

%init vectors
est.pitch_est = zeros(1,length(est.depth));
est.istate = zeros(1,length(est.depth));
est.pstate = zeros(1,length(est.depth));
est.iterm  = zeros(1,length(est.depth));
est.pterm  = zeros(1,length(est.depth)); 

%do index 1 calcs
if(est.depth_goal(1) == 0 && abs(est.depth(1)) < MTOF/2)
	est.pitch_est(1) = 5;
else
	est.istate(1)	= est.depth_err(1)*dt;
	est.istate(1)	= min(max(est.istate(1),istatemin),istatemax);
	est.pstate(1)	= est.depth_err(1);
	est.iterm(1)	= est.istate(1)*depth_ki;
	est.pterm(1)	= est.pstate(1)*depth_kp;
	est.pitch_est(1)= est.pterm(1) + est.iterm(1);
end
for i = 2:length(est.depth)
	if(est.depth_goal(i) == 0 && abs(est.depth(i)) < MTOF/2)
		%surface behavior gets fixed pitch reference
		est.pitch_est(i)= 5;
	else
		%depth behavior is PI control
		est.istate(i)	= est.istate(i-1) + est.depth_err(i)*dt;
	%	if ~(abs(est.depth_err(i))<MTOF/2)
			% if we're near the surface stop thresholding the integrator
			est.istate(i)	= min(max(est.istate(i),istatemin),istatemax);
	%	end
		est.pstate(i)	= est.depth_err(i);
		est.iterm(i)	= est.istate(i)*depth_ki;
		est.pterm(i)	= est.pstate(i)*depth_kp;
		est.pitch_est(i)= est.pterm(i) + est.iterm(i);
	end
end

est.pitch_est	= min(max(est.pitch_est,-anglerange),anglerange);

