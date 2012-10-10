function plink_t_publish (pair_t, sensor_id, cfg)

global lc

plink_channel      = cfg.channels.plink_t;

plink = perllcm.isam_plink_t ();
vlink = perllcm.isam_vlink_t ();

plink.utime_i = pair_t.id1;
plink.utime_j = pair_t.id2;
plink.prior   = 1;
plink.link_id = 0;

x_ji = perllcm.pose3d_t ();
for ii=1:6, x_ji.mu(ii) = pair_t.mu(ii); end
S_vec = reshape (pair_t.S, 1, 36);
for ii=1:36, x_ji.Sigma(ii) = S_vec(ii); end
plink.x_ji = x_ji;

if (sensor_id == cfg.sensor.camera)
    plink.sensor_id = vlink.SENSOR_CAMERA;
end

% send odometry meas
lc.publish(plink_channel, plink);