
//  Main Equation:
//  M x v_dot + C_RB(v_r)v_r + C_A(v_r)*v_r + D*v_r + d(v_r) = Tau_control + Tau_wind + Tau_wave

//Tau_control:
// X = (F_port + F_stbd - 2 * F_drag) * (u/sqrt(u^2_v^2));
// Y = (F_port + F_stbd - 2 * F_drag) * (v/sqrt(u^2_v^2));
// N = (F_port + F_stbd - 2 * F_drag) * (width/2);
// May remove F_drag from here and include it instead in the D and d terms
//
// Thrust to control signal Equation
// Two Signals: Port_CS and Stbd_CS 
// Range: -1000..0..+1000
// Corresponding Thrust Range: -840.714..0..+840.714 Newtons (189lbf source: Torqeedo Electric Boating 2016 Spec book) 
// F_port = Port_CS/1000 * 840.714 // Single Motor Thrust Force Newtons

// Vectors
// State components 3DOF (2D plane) TODO - vectors?
// Body fixed frame of reference
u =         // body fixed linear velocity in x = surge m/s
v =         // body fixed linear velocity in y = sway m/s
r =         // body fixed angular velocity in z = yaw = heading change rad/s
// Inertial Frame of Reference 3DOF (NED) 
N =         // North
E =         // East
psi =       // Heading (Euler Angle)

v_c = vector?       // current velocity - make this something simple, constant and non-rotational e.g. [u_c, v_c, r_c=0]
// TODO - but this is relative to the vehicle whereas we probably want it constant in 2D NED then convert
v_r = v - v_c       // vector u_r, v_r, r_r - relative velocity of the vehicle to the water

// Vehicle Measurements
draft_len = 3.10   // draft length length of wetted area measured front to rear m
payload_len = 2.90      // approx. length of payload mass m
beam = 2.4384   // outer width m
m_to_m_width = 2.02     // motor to motor width m measured at motor functional centre
hf = 0.17   // front (min) draft height (segment height) m 
rf = 0.10   // front (min) radius of tubular floats m
hr = 0.15;  // rear (max) draft height (segment height) m 
rr = 0.215;	// rear (max) radius of tubular floats m
cs_area_yz = r^2 * cos^-1((r-h)/r)-(r-h) * sqrt(2rh -h^2);	// max cross section area perpendicular to flow (per single tubular float) 
cs_area_xz = draft_len * (hf + 1/2(hr-hf));  // cross sectional area perpendicular to cross flow (per single tubular float)
al_f = rf * 2 * cos^-1(1-(hf/rf));   // arc length of wetted area at front of float
al_r = rr * 2 * cos^-1(1-(hr/rr));   // arc length of wetted area at rear of float 
S = draft_len * (al_f + 1/2(al_f-al_f));    // wetted area of a float treated as a flattened symetrical trapezoid
asv_mass = 181.437;     // kg from spec sheet 400lb
load_mass = 200;    // sum of known components is 188kg
C_D = 0.25;  // Drag co-efficient Source: FEA estimate g.wakeham 11/2017
rho = 1027;  // Water density kg/m^3

// Drag force for a single pontoon:  F_drag = 0.5 * area * C_D * rho * v^2
F_drag_mult_x = 0.5 * 2 * cs_area_yz * C_D * rho;  // Drag force = F_drag_mult_n * velocity^2
F_drag_mult_y = 0.5 * 2 * cs_area_xz * C_D * rho;  // using the same C_D here, as the side on radius is still pretty streamlined

// Non-State Matrix Components
m = asv_mass + load_mass;    // mass of asv kg + load mass kg
I_z = (1.0/12.0) * m (draft_len*draft_len + beam*beam);   // moment of inertia about the z axis - treat as rect. box since approx. weight dist.
x_g =       // vector component in x (fwd/aft) from origin of body O_b to centre of gravity CG
X_u_dot =   
Y_v_dot = 
Y_r_dot =
N_r_dot = 

// Matrices:

// M = M_RB + M_A Rigid Body Mass Matrix + Hydrodynamic Added Mass
SMALL::Matrix33 M =  m - X_u_dot, 		0, 						0,
			            0, 			m - Y_v_dot, 		m * x_g - Y_r_dot,
			            0, 			m * x_g - Y_r_dot		I_z - N_r_dot;

// C_RB(v_r) Coriolis Matrix (Rigid Body)
SMALL::Matrix33 C_RB_of_v_r =  0,               0,          -m(x_g * r + v),
                               0,               0,          m * u,
                               m(x_g * r + v),  -m * u,        0;   

// C_A(v_r) Coriolis Matrix (Added Mass)
SMALL::Matrix33 C_A_of_v_r =  0,                0,              Y_v_dot * v_r + Y_r_dot * r,
                              0,                0,              -X_u_dot * u_r,
               -Y_v_dot * v_r - Y_r_dot * r,    X_u_dot * u_r,         0;

// D Linear damping terms (low speed and stationkeeping)
SMALL::Matrix33 D =  -X_u,    0,     0,
                       0,   -Y_v    -Y_r,
                       0,   -N_v    -N_r;

// d(v_r) Velocity based damping terms (higher speeds)
// d(v_r) transpose = 3x1 matrix: 1/2*rho*S(1+k)C_f(U_r)*|U_r|*U_r, ... refer eqn 7.20, 6.91, 6.92 Fossen 2011
// Simpler Temporary approach:
//
//

// Hydrodynamic added mass terms 
// X_u_dot =        // del_X/del_u_dot Partial derivative of X wrt u_dot
// Y_v_dot =        // del_Y/del_v_dot Partial derivative of Y wrt v_dot
// Y_r_dot =        // del_Y/del_r_dot Partial derivative of Y wrt r_dot
// N_r_dot =        // del_Y/del_r_dot Partial derivative of N wrt r_dot



