function nodes_t = isam_pose_parser(filename, N)
% function nodes_t = isam_pose_parser(filename, N)
% parse 'graph.isam' file read Pose3d_Node line and create matlab TheJournal
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    2011.09.23      ak          Moved from matlab branch / created

% do we know N in advance somehow?
[mu, sigma, x_vc, utime, idx] = deal([]);
if (nargin == 3)
    mu = zeros (6, N);
    sigma = zeros (36, N);
    x_vc = zeros (6, N);
    utime = zeros (N,1);
end
    

nodeline = 'Pose3d_Node';
len_nodeline = length(nodeline);
fid = fopen (filename);

if (fid > 0)
    tline = fgets(fid);
    while ischar(tline)
        line_title = tline(1:len_nodeline);
        if (strcmp (line_title, nodeline))
            % fprintf (1,'%s',tline);
            [header, pose, utime_offset_tmp, x_vs, blank, sigma_tmp] = strread (tline, '%s %s %s %s %s %s','delimiter','(){}');
            [prefix, idx_tmp] = strread (char(header), '%s %d');
            fni = idx_tmp + 1; % isam is 0 based
            
            % idx
            idx(fni) = idx_tmp;
            
            % utime
            [utime_tmp, offset] = strread(char(utime_offset_tmp),'%s %s');
            
            valid_cam_node = ~isempty(char(utime_tmp)) && ~isempty(char(offset)) && ~isempty(char(sigma_tmp));
            valid_node = ~isempty(char(utime_tmp));
            
            if (valid_node)
                utime(fni) = str2double(char(utime_tmp));
            else
                utime(fni) = 0;
            end
            
            % mu
            [x,y,z,h,p,r] = strread (char(pose), '%f %f %f %f %f %f','delimiter',',;'); 
            mu(:,fni) = [x,y,z,r,p,h]';

            % sqrt information
            if (valid_cam_node)
                a = strread (char(sigma_tmp),'%f','delimiter',',');
                sigma(:,fni) = a;
            else
                sigma(:,fni) = zeros(36,1);
            end
            
            % sensor coordinate transform
            if (~isempty(char(x_vs)))                
                [x,y,z,h,p,r] = strread (char(x_vs), '%f %f %f %f %f %f','delimiter',',;'); 
                x_vc(:,fni) = [x,y,z,r,p,h]';
            else
                x_vc(:,fni) = zeros(6,1);
            end
            
        end
        tline = fgets(fid);
    end
else
    fprintf (1,'Error opening %s\n',filename);
end

fclose (fid);

valid_idx = find(sum(mu,1) ~= 0);

% construct nodes_t and return
nodes_t.n = length(valid_idx);
nodes_t.mu = mu(:,valid_idx);
nodes_t.sigma = sigma(:,valid_idx);
nodes_t.x_vc = x_vc(:,valid_idx);
nodes_t.utime = utime(:,valid_idx);
nodes_t.idx = idx(:,valid_idx);

idx = 1;
for i=valid_idx
    if (utime(i) > 0)
        nodes_t.imgfile{idx} = sprintf ('%16.f.tif',utime(i));
    end
    idx = idx + 1;
end
