#include "platform_utils.h"

perls::Input compute_uvwrph_odo (int64_t dt, double sig_d, double sig_psi, 
        const Eigen::Matrix<double,6,1>& uvwrph)
{
    perls::Input in;

    double dt_secs = dt*MICROSEC_TO_SEC;

    Eigen::Vector2d uv; 
    uv << uvwrph(0), uvwrph(1);
    Eigen::Matrix2d UV;
    UV << uv(0)*uv(0), uv(0)*uv(1),
          uv(0)*uv(1), uv(1)*uv(1);

    Eigen::Vector3d rph = uvwrph (rph_i);
    Eigen::Matrix2d R;
    R << cos (rph(2)), -sin (rph(2)),
         sin (rph(2)),  cos (rph(2));
    Eigen::Matrix2d R_p;
    R_p << -sin (rph(2)), -cos (rph(2)),
            cos (rph(2)), -sin (rph(2));

    Eigen::Matrix2d Sig_d = (sig_d*sig_d) * Eigen::Matrix2d::Identity ();

    in.input_raw = uv * dt_secs;
    in.input_cov = (dt_secs*dt_secs) * R*Sig_d*R.transpose ()
        + (dt_secs*dt_secs) * R_p * (UV + Sig_d) * R_p.transpose () * (sig_psi*sig_psi);
    return in;
}
