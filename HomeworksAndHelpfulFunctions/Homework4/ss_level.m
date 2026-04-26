function [A_lon,B_lon,A_lat,B_lat] = ss_level(x_td,params)
%Author: Lucas Becker
%Date: 2/2/2026
%ASEN 5128 Homework4 function
%Take in inputs as trim definition for STRAIGHT WINGS-LEVEL FLIGHT and an
%aircraft parameters structure

%calculating full aircraft state and control surfaces from homework3
%functions
[aircraft_state_trim, control_surface_trim, ~] = calculate_trim_level(x_td,params);

g = 9.807;  %gravity constant in m/s62
rho = stdatmo(-aircraft_state_trim(3));

%extracting euler angles:
phi = aircraft_state_trim(4);
theta = aircraft_state_trim(5);
psi = aircraft_state_trim(6);

%extracting velocities:
u = aircraft_state_trim(7);
v = aircraft_state_trim(8);
w = aircraft_state_trim(9);

%extracting control inputs, aileron and rudder zero for straight level
%flight
del_e = control_surface_trim(1);
del_t = control_surface_trim(4);



%calculating wind angles
Va = x_td(1);
alpha = theta - x_td(2);
beta = 0;

%Extracting some aircraft parameters for easier typing:
S = params.S;
m =  params.m;
c = params.c;
S_prop = params.Sprop;
C_prop = params.Cprop;
b = params.b;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Extracting Non Dimensional Stability Derivatives from params for easier
%debugging:
C_L_0 = params.CL0;
C_m_o = params.Cm0;
C_D_min = params.CDmin;
C_L_min = params.CLmin;

C_L_alpha = params.CLalpha;
C_m_alpha = params.Cmalpha;
C_L_q = params.CLq;
C_m_q = params.Cmq;

C_L_de = params.CLde;
C_m_de = params.Cmde;

C_Y_beta = params.CYbeta;
C_l_beta = params.Clbeta;
C_n_beta = params.Cnbeta;

C_Y_p = params.CYp;
C_l_p = params.Clp;
C_n_p = params.Cnp;

C_l_r = params.Clr;
C_n_r = params.Cnr;
C_Y_r = params.CYr;

C_Y_da = params.CYda;
C_l_da = params.Clda;
C_n_da = params.Cnda;

C_Y_dr = params.CYdr;
C_l_dr = params.Cldr;
C_n_dr = params.Cndr;

C_p_o = params.Cp0;
C_p_beta = params.Cpbeta;

C_p_p = params.Cpp;
C_p_r = params.Cpr;

C_p_da = params.Cpda;
C_p_dr = params.Cpdr;


C_r_o = params.Cr0;
C_r_beta = params.Crbeta;

C_r_p = params.Crp;
C_r_r = params.Crr;

C_r_da = params.Crda;
C_r_dr = params.Crdr;

km = params.kmotor;



%inertia term:
J_y = params.Iy;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% LONGITUDINAL %%%

%determining CL and CD:
CL = C_L_0 + C_L_alpha * alpha + C_L_de * control_surface_trim(1);  %lift coefficient
CD = C_D_min + params.K*(CL - C_L_min)^2;   %drag coefficient

%converting to CX and CZ
C_X = -CD * cos(alpha) + CL*sin(alpha);
C_Z = -CD * sin(alpha) - CL*cos(alpha);

%CD derivatives
C_D_alpha = 2*params.K*(CL - C_L_min)*C_L_alpha;
C_D_q     = 2*params.K*(CL - C_L_min)*C_L_q;
C_D_de    = 2*params.K*(CL - C_L_min)*C_L_de;


%Calculating Longitudinal Non Dimensional Derivatives:
C_X_q = -C_D_q*cos(alpha) + C_L_q * sin(alpha);
C_X_alpha = -C_D_alpha * cos(alpha) + CD*sin(alpha) + C_L_alpha * sin(alpha) + CL*cos(alpha);
C_X_de = -C_D_de * cos(alpha) + C_L_de * sin(alpha);
C_Z_q = -C_D_q * sin(alpha) - C_L_q * cos(alpha);
C_Z_de = -C_D_de * sin(alpha) - C_L_de * cos(alpha);
C_Z_alpha = -C_D_alpha*sin(alpha) - CD * cos(alpha) - C_L_alpha * cos(alpha) + CL * sin(alpha);

%Dimensional derivatives:
X_u = (u * rho * S / m) * C_X - (rho * S * w * C_X_alpha / (2*m)) +  (rho * S_prop * C_prop * del_t / m) * ((km * u / Va) * (1 - 2*del_t) + 2 * u * (del_t - 1));
X_w = (w * rho * S / m) * C_X + rho * S * C_X_alpha * u / (2 * m) + (rho * S_prop * C_prop * del_t / m) * ((km * w / Va) * (1 - 2*del_t) + 2 * w * (del_t - 1));
X_q = -w + rho * Va * S * C_X_q * c / (4 * m);
X_de = rho * (Va^2) * S * C_X_de / (2 * m);
X_dt = (rho * S_prop * C_prop / m) * (Va * (km - Va) + 2 * del_t * (km - Va)^2);
Z_u = (u * rho * S / m) * C_Z - (rho * S * C_Z_alpha * w / (2 * m));
Z_w = (w * rho * S / m) * C_Z + (rho * S * C_Z_alpha * u / (2 * m));
Z_q = u + (rho * Va * S * C_Z_q * c / (4 * m));
Z_de = rho * (Va^2) * S * C_Z_de / (2 * m);
M_u = (u * rho * S * c / J_y) * (C_m_o + C_m_alpha * alpha + C_m_de * del_e) - (rho * S * c * C_m_alpha * w / (2 * J_y));
M_w = (w * rho * S * c / J_y) * (C_m_o + C_m_alpha * alpha + C_m_de * del_e) + (rho * S * c * C_m_alpha * u / (2 * J_y));
M_q = rho * Va * S * (c^2) * C_m_q / (4 * J_y);
M_de = rho * (Va^2) * S * c * C_m_de / (2 * J_y);


A_lon = [X_u, X_w * Va * cos(alpha), X_q, -g*cos(theta), 0;
         Z_u/(Va * cos(alpha)), Z_w, Z_q / (Va * cos(alpha)), -g * sin(theta) / (Va * cos(alpha)), 0;
         M_u, M_w * Va * cos(alpha), M_q, 0, 0;
         0, 0, 1, 0, 0;
         sin(theta), -Va * cos(theta) * cos(alpha), 0, u * cos(theta) + w * sin(theta), 0];

B_lon = [X_de, X_dt;
         Z_de/(Va * cos(alpha)), 0;
         M_de, 0;
         0, 0;
         0, 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% LATERAL %%%

%stability derivatives:
Y_v = (rho * S * C_Y_beta / (2*m)) * Va;
Y_p = w + (rho * Va * S * b * C_Y_p / (4*m));
Y_r = -u + (rho * Va * S * b * C_Y_r / (4*m));
Y_da = rho * (Va^2) * S * C_Y_da / (2*m);
Y_dr = rho * (Va^2) * S * C_Y_dr / (2*m);
L_v =  (rho * S * b * v) * (C_p_o) + (rho * S * b * C_p_beta * Va / (2));
L_p = (rho * Va * S * b^2 * C_p_p / 4);
L_r = (rho * Va * S * b^2 * C_p_r / (4));
L_da = rho * (Va^2) * S * b * C_p_da / (2);
L_dr = rho * (Va^2) * S * b * C_p_dr / (2);
N_v = (rho * S * b * v ) * (C_r_o) + (rho * S * b * C_r_beta / (2)) * Va;
N_p = (rho * Va * S * b^2 * C_r_p / (4));
N_r = (rho * Va * S * b^2 * C_r_r / (4));
N_da = rho * (Va^2) * S * b * C_r_da / (2);
N_dr = rho * (Va^2) * S * b * C_r_dr / (2);



A_lat = [Y_v, Y_p/(Va * cos(beta)), Y_r/(Va * cos(beta)), g * cos(theta) * cos(phi) / (Va * cos(beta)), 0;
         L_v * Va * cos(beta), L_p, L_r, 0, 0;
         N_v * Va * cos(beta), N_p, N_r, 0, 0;
         0, 1, tan(theta), 0, 0;
         0, 0, sec(theta), 0, 0];

B_lat = [Y_da/Va, Y_dr/Va;
         L_da, L_dr;
         N_da, N_dr;
         0, 0;
         0, 0];





end