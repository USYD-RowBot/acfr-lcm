function nav_t = mkrenav(nav_t,ssv_t);
%function renav_t = mkrenav(nav_t,ssv_t);

nav_t.EKF.rovtime = ssv_t.t';
nav_t.EKF.abstime = nav_t.EKF.rovtime+nav_t.STARTTIME;
nav_t.EKF.nx      = ssv_t.mu_x(2,:)';
nav_t.EKF.ny      = ssv_t.mu_x(1,:)';
nav_t.EKF.nz      = ssv_t.mu_x(3,:)';
nav_t.EKF.roll    = ssv_t.mu_x(4,:)';
nav_t.EKF.pitch   = ssv_t.mu_x(5,:)';
nav_t.EKF.heading = ssv_t.mu_x(6,:)';
