double xG = 0;
double yG = 0;
double zG = 0;
double m = 30;
double tt_horiz_fore = 7.000000e-01;
double tt_horiz_aft = -6.000000e-01;
double tt_vert_fore = 6.000000e-01;
double tt_vert_aft = -5.000000e-01;
double Ix = 8.437500e-02;
double Ixx = 8.437500e-02;
double Iy = 1.264225e+01;
double Iyy = 1.264225e+01;
double Iz = 1.264225e+01;
double Izz = 1.264225e+01;
double W = 3.001860e+02;//2.943000e+02;
double B = 3.001860e+02;
double Xuu = -3.906355e+00;
double Yvv = -2.013647e+02;
double Mww = 1.971290e+01;
double Yrr = 6.050947e-01;
double Mqq = -6.546001e+01;
double Zww = -2.013647e+02;
double Nvv = -1.971290e+01;
double Zqq = -6.050947e-01;
double Nrr = -6.546001e+01;
double Kpp = -1.529807e-01;
double Xudot = -5.641763e-01;
double Yvdot = -6.421909e+01;
double Mwdot = 2.703423e+01;
double Mqdot = -4.253427e+01;
double Zwdot = -6.421909e+01;
double Nvdot = -2.703423e+01;
double Yrdot = -2.703423e+01;
double Zqdot = 2.703423e+01;
double Nrdot = -4.253427e+01;
double Kpdot = -3.534732e-01;
double Xwq = -6.421909e+01;
double Xqq = 2.703423e+01;
double Xvr = 6.421909e+01;
double Xrr = 2.703423e+01;
double Yura = -5.641763e-01;
double Ywp = 6.421909e+01;
double Ypq = -2.703423e+01;
double Zuqa = 5.641763e-01;
double Zvp = 2.703423e+01;
double Zrp = -2.703423e+01;
double Muwa = 6.365491e+01;
double Mvp = 2.703423e+01;
double Mrp = 4.218079e+01;
double Muqa = -2.703423e+01;
double Nuva = -6.365491e+01;
double Nwp = 2.703423e+01;
double Npq = -4.218079e+01;
double Nura = -2.703423e+01;
double Ydr = 7.977946e+01;
double Zdp = -7.977946e+01;
double Mdp = -8.297064e+01;
double Ndr = -8.297064e+01;
double Yuv = -7.978854e+01;
double Yur = 8.240647e+01;
double Zuw = -7.978854e+01;
double Zuq = -8.240647e+01;
double Muw = -1.930248e+01;

double Muq = -1.133237e+02;
double Nuv = 1.930248e+01;
double Nur = -1.133237e+02;

Matrix inv_inertia(6,6);

void populate_inv_inertia()
{
    inv_inertia(0,0) = 3.271804e-02;
    inv_inertia(0,1) = 0;
    inv_inertia(0,2) = 0;
    inv_inertia(0,3) = 0;
    inv_inertia(0,4) = 0;
    inv_inertia(0,5) = 0;
    inv_inertia(1,0) = 0;
    inv_inertia(1,1) = 1.234973e-02;
    inv_inertia(1,2) = 0;
    inv_inertia(1,3) = 0;
    inv_inertia(1,4) = 0;
    inv_inertia(1,5) = -6.050860e-03;
    inv_inertia(2,0) = 0;
    inv_inertia(2,1) = 0;
    inv_inertia(2,2) = 1.234973e-02;
    inv_inertia(2,3) = 0;
    inv_inertia(2,4) = 6.050860e-03;
    inv_inertia(2,5) = 0;
    inv_inertia(3,0) = 0;
    inv_inertia(3,1) = 0;
    inv_inertia(3,2) = 0;
    inv_inertia(3,3) = 2.283897e+00;
    inv_inertia(3,4) = 0;
    inv_inertia(3,5) = 0;
    inv_inertia(4,0) = 0;
    inv_inertia(4,1) = 0;
    inv_inertia(4,2) = 6.050860e-03;
    inv_inertia(4,3) = 0;
    inv_inertia(4,4) = 2.108833e-02;
    inv_inertia(4,5) = 0;
    inv_inertia(5,0) = 0;
    inv_inertia(5,1) = -6.050860e-03;
    inv_inertia(5,2) = 0;
    inv_inertia(5,3) = 0;
    inv_inertia(5,4) = 0;
    inv_inertia(5,5) = 2.108833e-02;
}
