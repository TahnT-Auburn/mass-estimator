%% Vehicle Parameters - Tractor 
function [m_tract, m_trail, n_trans, n_diff, n_conv, cd, r_eff, front_area, f_rr_c, i_e, i_t, i_ds, ...
         i_diff, i_wheel, b_e, b_t, b_diff] = TractTrailParams(gear)

% Author=
%           Tahn Thawainin, AU GAVLAB
%
% Description=
%           A function to store A2 Peterbilt vehicle parameters
%
% Inputs=
%           gear (1-9) - gear setting during the run (determines n_trans value)
%
% Outputs=
%           m_tract - tractor mass (kg)
%           m_trail - trailer mass (kg)
%           n_trans - transmission gear ratio
%           n_diff - differential gear ratio
%           n_conv - torque converter gear ratio
%           cd - coefficient of drag
%           r_eff - effective rolling radius (m)
%           front_area - tractor front surface area (m^2)
%           f_rr_c - rolling resistance coefficient 
%           
%           i_e - engine inertia (kg-m^2)
%           i_t - transmission inertia (kg-m^2)
%           i_ds - ds inertia (kg-m^2)
%           i_diff - differential inertia (kg-m^2)
%           i_wheel - wheel inertia (kg-m^2)
%
%           b_e - engine damping (Ns^2/m^2)
%           b_t - transmission damping (Ns^2/m^2)
%           b_diff - differential damping (Ns^2/m^2)

m_tract = 8892;
m_trail = 4526;
n_trans = [11.06 , 10.2 , 7.062 , 4.984 , 3.966 , 2.831, 2.03, 1.47, 1, 0.74];
n_diff = 4.4;
n_conv = 1;
cd = 0.79;
r_eff = 0.510;
front_area = 7.8016;
f_rr_c = 0.0041;

i_e = 2.75;
i_t = 0.13;
i_ds = 0.012;
i_diff = 0.028;
i_wheel = 1700;

b_e = 2.21;
b_t = 1.4;
b_diff = 9.7;

% return n_trans from selected gear
n_trans = n_trans(gear);

end