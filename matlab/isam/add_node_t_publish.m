function add_node_t_publish (utime, idx, mu, cfg)

global lc

add_node_channel      = cfg.channels.add_node_t;

add_node_t = perllcm.isam_add_node_t();
add_node_t.id = idx;
add_node_t.utime = utime;
add_node_t.node_type = add_node_t.NODE_POSE3D;
add_node_t.sensor_id = add_node_t.NODE_NOSENSOR;
if (sum(mu) ~=0)
    add_node_t.has_mu_o = 1;
    add_node_t.mu_o(1) = mu(1);
    add_node_t.mu_o(2) = mu(2);
    add_node_t.mu_o(3) = mu(3);
    add_node_t.mu_o(4) = mu(4);
    add_node_t.mu_o(5) = mu(5);
    add_node_t.mu_o(6) = mu(6);    
else
    add_node_t.has_mu_o = 0;
end
lc.publish (add_node_channel, add_node_t);