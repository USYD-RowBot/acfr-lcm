function isam_client (filename)
% isam_client to communicate with isam-server
%
%  NOTE: before calling this function, make sure you've added jars needed
%
%  import java.io.*;
%  eval(['javaaddpath ','${PERLS_DIR}/third-party/build/{LCMDIR}/lcm-java/lcm.jar']);
%  eval(['javaaddpath ','${PERLS_DIR}/lib/perllcm.jar']);
%
%  Ayoung Kim, 2011.09.22

% make java lcm object global ans shared between function
global lc

% generate nodes_t and meas_t from the file
[nodes_t, meas_t] = isam2mat (filename, 0, 0);

% channel name
cfg.channels.add_node_t      = 'ADD_NODE';
cfg.channels.add_node_ack_t  = 'ADD_NODE_ACK';
cfg.channels.st_return_t     = 'RETURN_STATE';
cfg.channels.st_rq_t         = 'REQUEST_STATE';
cfg.channels.vlink_t         = 'VLINKS';
cfg.channels.plink_t         = 'PLINKS';
cfg.sensor.odometry          = 1;
cfg.sensor.camera            = 2;

%% send / receive msg
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
millis_to_wait = 1000;

%% load nodes_t and meas_t
n = nodes_t.n;

for ii=1:n
    utime = nodes_t.utime(ii);
    if (utime == 0), continue; end;

    % add node    
    idx = nodes_t.idx(ii);    
    mu = nodes_t.mu(:,ii);
    add_node_t_publish (utime, idx, mu, cfg);

    if (nodes_t.idx(ii) == 0), continue; end;

    % need hand shaking by ack message    
    pause (0.1);

    % add odometry constaint
    odo_idx = find (meas_t.odo_time(2,:) == utime);
    if (~isempty (odo_idx))
        odo_t = meas_t.odo_t{odo_idx};
        vlink_t_publish (odo_t, cfg.sensor.odometry, cfg);
    else
        display ('Error: missing odometry!\n');
    end
    
    % add camera constraint
    cam_idx = find (meas_t.cam_time(2,:) == utime);
    if (~isempty (cam_idx))
        cam_t = meas_t.cam_t{cam_idx};
        vlink_t_publish (cam_t, cfg.sensor.camera, cfg);
    end
    
%     % wait for ack msg
%     node_ack_wait = 1;    
%     while (node_ack_wait)
%         lc.subscribe(add_node_ack_channel, aggregator);
%         msg = aggregator.getNextMessage(millis_to_wait);
%         if length(msg) > 0
%             display ('received\n');
%             node_ack_wait = 0;
%         end    
%     end
%     % wait for state to return
%     state_wait = 1;
%     while (state_wait)
%         lc.subscribe(state_return_channel, aggregator);
%         msg = aggregator.getNextMessage(millis_to_wait);
%         if length(msg) > 0
%             display ('received\n');
%             state_wait = 0;
%         end    
%     end
end
