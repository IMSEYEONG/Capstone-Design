clear all, clc;

La = 1.526*10^-9;                 % Inductance [H]
Va = 24;                          % Input Voltage [V]
Ra = 0.218;                       % Resistance [ohm]                                    
Kt = 0.0531;                      % Torque Constant [Nm/A]
Ke = 0.0531;                      % Back EMF Constant [Nm/A]
max_spd = 3000*(30/pi);           % maximum spped [rad/s]
max_cur = 12;                     % maximum current [A]
Jm  = 500*(10^-3)*(10^-2)^2;      % Rotor Inertia 
% Jg = ;                          % Gear inertia
% t = ;                           % Mechanical Time Constant
% bm  = ;                         % Friction Coefficient(J/t) 
% bg = ;                          % Gear Friction Coefficient
n = 20;                         % Gear Ratio
% a = ;                           % Gear Efficiency  

% k = 1/a*(1/n)^2;
% 
% Jeq = Jm + k*Jg + k*J_load;         % Jeq
% Beq = bm + k*bg + k*B_load;         % Jeq/tau;

m = 0;                           % [kg]
Loadtorq = m*9.81*0.01/20;              % [Nm]

% Q-filter
wa = 10; 
tau = 1;
% current controller
fcc = 200;
Wcc = fcc*2*pi;
Kp_cur = La * Wcc;
Ki_cur = Ra * Wcc;
Ka_cur = 1/Kp_cur;
% speed controller
Wcs = Wcc / 10;
Kp_spd = Jm*Wcs/Ke;
Ki_spd = Kp_spd*Wcs/5;
Ka_spd = 1/Kp_spd;
% position controller
Wcp = Wcs / 10;
Kp_pos = Wcp;
Kd_pos = Wcp/Wcs;
deg_rep = 90;