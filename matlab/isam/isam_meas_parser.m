function [odo_t, cam_t, sonar_t, odo_time, cam_time, sonar_time, odo_idx, cam_idx, sonar_idx] =...
          isam_meas_parser (filename, N)
% function [odo_t, cam_t, sonar_t, odo_time, cam_time, sonar_time] = isam_meas_parser (filename, N)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2011.09.23      ak          Moved from matlab branch / created
%    2011.10.04      ak          Added timestamp

[odo_t odo_time, odo_idx] = parse_isam_odo_meas (filename);
[cam_t cam_time, cam_idx] = parse_isam_cam_meas (filename);
[sonar_t sonar_time, sonar_idx] = parse_isam_sonar_meas (filename);

function [odo_t, odo_timestamp, odo_idx, meas_idx] = parse_isam_odo_meas (filename)

meas_idx = 1;
odo_t = [];
odo_timestamp = [];
odo_idx = [];
fid = fopen (filename,'rt');

odoline = 'Pose3d_Pose3d_Factor';
if (~fid), return; end

tline = fgets(fid);
while ischar(tline)
    nodeline = tline(1:length(odoline));
    if (strcmp (odoline, nodeline))
        [header, z_tmp, p3, R_info_tmp, utimes] = strread (tline, '%s %s %s %s %s','delimiter','(){}');
        [prefix, idxtmp_i, idxtmp_j] = strread (char(header), '%s %d %d');
        fni = idxtmp_i+1; fnj = idxtmp_j+1; % isam is 0 based
        
        % z
        [x,y,z,h,p,r] = strread (char(z_tmp),'%f %f %f %f %f %f','delimiter',',;');
        z_meas = [x,y,z,h,p,r]';
        
        % R
        l = strread (char(R_info_tmp),'%f','delimiter',',');
        L(1:6,1) = l(1:6);   L(1,1:6) = l(1:6);
        L(2:6,2) = l(7:11);  L(2,2:6) = l(7:11);
        L(3:6,3) = l(12:15); L(3,3:6) = l(12:15);
        L(4:6,4) = l(16:18); L(4,4:6) = l(16:18);
        L(5:6,5) = l(19:20); L(5,5:6) = l(19:20);        
        L(6,6) = l(21);
        R_meas = inv(L.^2);
                
        % utime i and j
        [utime_i, utime_j] = strread (char(utimes),'%16.f %16.f');
        if (isempty(utime_i)), utime_i = 0; end;
        if (isempty(utime_j)), utime_j = 0; end;
        
        % put in meas_t
        meas_t.fni = fni;
        meas_t.fnj = fnj;
        meas_t.z = z_meas;
        meas_t.R = R_meas;
        meas_t.utime_i = utime_i;
        meas_t.utime_j = utime_j;
        meas_t.idx_i = idxtmp_i;
        meas_t.idx_j = idxtmp_j;
        
        odo_t{meas_idx} = meas_t;
        odo_timestamp(1,meas_idx) = utime_i;
        odo_timestamp(2,meas_idx) = utime_j;        
        odo_idx(1,meas_idx) = idxtmp_i;
        odo_idx(2,meas_idx) = idxtmp_j;        
        meas_idx = meas_idx + 1;
    end
    tline = fgets(fid);
end

% check if the file has utime information
if (sum(odo_timestamp) == 0), odo_timestamp = []; end

fclose(fid);

function [cam_t, cam_timestamp, cam_idx, meas_idx] = parse_isam_cam_meas (filename)

meas_idx = 1;
cam_t = [];
cam_timestamp =[];
cam_idx = [];
fid = fopen (filename,'rt');

camline = 'Pose3db_Pose3db_Factor';
if (~fid), return; end

tline = fgets(fid);
while ischar(tline)
    nodeline = tline(1:length(camline));
    if (strcmp (camline, nodeline))
        [header, z_tmp, p3, R_info_tmp, p5, x_vc_i_tmp, p7, x_vc_j_tmp, utimes] = strread (tline, '%s %s %s %s %s %s %s %s %s','delimiter','(){}');
        [prefix, idxtmp_i, idxtmp_j] = strread (char(header), '%s %d %d');
        fni = idxtmp_i+1; fnj = idxtmp_j+1; % isam is 0 based
        
        % z
        [a,e,h,p,r] = strread (char(z_tmp),'%f %f %f %f %f','delimiter',',;');
        z_meas = [a,e,r,p,h]';
        
        % R
        l = strread (char(R_info_tmp),'%f','delimiter',',');
        L(1:5,1) = l(1:5);   L(1,1:5) = l(1:5);
        L(2:5,2) = l(6:9);   L(2,2:5) = l(6:9);
        L(3:5,3) = l(10:12); L(3,3:5) = l(10:12);
        L(4:5,4) = l(13:14); L(4,4:5) = l(13:14);
        L(5,5) = l(15);
        R_meas = inv(L.^2);
        
        % sensor xform
        [x,y,z,h,p,r] = strread (char(x_vc_i_tmp), '%f %f %f %f %f %f','delimiter',',;'); 
        x_vc_i = [x,y,z,r,p,h]';
        [x,y,z,h,p,r] = strread (char(x_vc_j_tmp), '%f %f %f %f %f %f','delimiter',',;'); 
        x_vc_j = [x,y,z,r,p,h]';
        
        % utime i and j
        [utime_i, utime_j] = strread (char(utimes),'%16.f %16.f');
        if (isempty(utime_i)), utime_i = 0; end;
        if (isempty(utime_j)), utime_j = 0; end;
        
        % put in meas_t
        meas_t.fni = fni;
        meas_t.fnj = fnj;
        meas_t.z = z_meas;
        meas_t.R = R_meas;
        meas_t.x_vc_i = x_vc_i;
        meas_t.x_vc_j = x_vc_j;
        meas_t.utime_i = utime_i;
        meas_t.utime_j = utime_j;
        meas_t.idx_i = idxtmp_i;
        meas_t.idx_j = idxtmp_j;
        
        cam_t{meas_idx} = meas_t;
        cam_timestamp(1,meas_idx) = utime_i;
        cam_timestamp(2,meas_idx) = utime_j;        
        cam_idx(1,meas_idx) = idxtmp_i;
        cam_idx(2,meas_idx) = idxtmp_j;        
        
        meas_idx = meas_idx + 1;
    end
    tline = fgets(fid);
end

% check if the file has utime information
if (sum(cam_timestamp) == 0), cam_timestamp = []; end

fclose(fid);

function [sonar_t, sonar_timestamp, sonar_idx, meas_idx] = parse_isam_sonar_meas (filename)

meas_idx = 1;
sonar_t = [];
sonar_timestamp = [];
sonar_idx = [];
fid = fopen (filename,'rt');

sonarline = 'Sonar2d_Factor';
if (~fid), return; end

tline = fgets(fid);
while ischar(tline)
    nodeline = tline(1:length(sonarline));
    if (strcmp (sonarline, nodeline))
        [header, z_tmp, p3, R_info_tmp, p5, x_vc_i_tmp, p7, x_vc_j_tmp, utimes] = strread (tline, '%s %s %s %s %s %s %s %s %s','delimiter','(){}');
        [prefix, idxtmp_i, idxtmp_j] = strread (char(header), '%s %d %d');
        fni = idxtmp_i+1; fnj = idxtmp_j+1; % isam is 0 based
        
        % z
        [dx,dy,dz] = strread (char(z_tmp),'%f %f %f','delimiter',',;');
        z_meas = [dx,dy,dz]';
        
        % R
        l = strread (char(R_info_tmp),'%f','delimiter',',');
        L(1:3,1) = l(1:3);   L(1,1:3) = l(1:3);
        L(2:3,2) = l(4:5);   L(2,2:3) = l(4:5);
        L(3,3) = l(6);
        R_meas = inv(L.^2);
        
        % sensor xform
        [x,y,z,h,p,r] = strread (char(x_vc_i_tmp), '%f %f %f %f %f %f','delimiter',',;'); 
        x_vc_i = [x,y,z,r,p,h]';
        [x,y,z,h,p,r] = strread (char(x_vc_j_tmp), '%f %f %f %f %f %f','delimiter',',;'); 
        x_vc_j = [x,y,z,r,p,h]';
        
        % utime i and j
        [utime_i, utime_j] = strread (char(utimes),'%16.f %16.f');
        if (isempty(utime_i)), utime_i = 0; end;
        if (isempty(utime_j)), utime_j = 0; end;

        % put in meas_t
        meas_t.fni = fni;
        meas_t.fnj = fnj;
        meas_t.z = z_meas;
        meas_t.R = R_meas;
        meas_t.x_vc_i = x_vc_i;
        meas_t.x_vc_j = x_vc_j;
        meas_t.utime_i = utime_i;
        meas_t.utime_j = utime_j;
        meas_t.idx_i = idxtmp_i;
        meas_t.idx_j = idxtmp_j;        

        sonar_t{meas_idx} = meas_t;
        sonar_timestamp(1,meas_idx) = utime_i;
        sonar_timestamp(2,meas_idx) = utime_j;        
        sonar_idx(1,meas_idx) = idxtmp_i;
        sonar_idx(2,meas_idx) = idxtmp_j;        
        
        meas_idx = meas_idx + 1;
    end
    tline = fgets(fid);
end

% check if the file has utime information
if (sum(sonar_timestamp) == 0), sonar_timestamp = []; end

fclose(fid);
