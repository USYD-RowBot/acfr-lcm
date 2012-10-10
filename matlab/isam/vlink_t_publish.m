function vlink_t_publish (meas_t, sensor_id, cfg)

global lc

vlink_channel      = cfg.channels.vlink_t;

vlink = perllcm.isam_vlink_t ();
vlink.id1 = meas_t.idx_i;
vlink.id2 = meas_t.idx_j;
vlink.link_id = 0;
%vlink.publisher_id = 0;
vlink.n = length(meas_t.z);
for ii=1:length(meas_t.z), vlink.z(ii) = meas_t.z(ii); end
R_vec = reshape (meas_t.R, 1, length(meas_t.z)*length(meas_t.z));
vlink.n2 = length(R_vec);
for ii=1:length(R_vec), vlink.R(ii) = R_vec(ii); end
vlink.accept = 1;
vlink.accept_code = vlink.CODE_ACCEPTED;

% sensor transform
if (isfield(meas_t,'x_vc_i') && isfield(meas_t,'x_vc_j'))
    vlink.dynamic_xvs = 1;    
    for ii=1:6, vlink.x_vs1(ii) = meas_t.x_vc_i(ii); end;
    for ii=1:6, vlink.x_vs2(ii) = meas_t.x_vc_j(ii); end;    
end

if (sensor_id == cfg.sensor.odometry)
    vlink.link_type = vlink.LINK_POSE3D;
    vlink.sensor_id = vlink.SENSOR_ODOMETRY;
elseif (sensor_id == cfg.sensor.camera)
    vlink.link_type = vlink.LINK_POSE3DB;
    vlink.sensor_id = vlink.SENSOR_CAMERA;
elseif (sensor_id == cfg.sensor.prior)
    vlink.link_type = vlink.LINK_PRIOR;
end

% send odometry meas
lc.publish(vlink_channel, vlink);
