%% Longitudinal Forces
function [Flon1, Flon2, Flon3] = lonForces(Ax, Aeng, Vx, torque_e, gear, grade)

% Author:
%           Tahn Thawainin, AU GAVLAB
%
% Description:
%           A function to calculate the longitudinal forces of a vehicle
%           based on a simple longitudinal force model
%
% Inputs:
%           Ax - longitudinal acceleration (m/s^2)
%           Aeng - engine angular acceleration (rad/s^2)
%           Vx - longitudinal velocity (m/s)    
%           torque_e - engine torque (N-m)
%           gear - current gear
%           grade - road grade (rad)
%
% Outputs:
%           Fdrive - longitudinal driving force (N)
%           Frr - longitudinal rolling resistance force (N)
%           Fdrag - longitudinal drag force (N)

%% Load Vehicle Parameters

[m_tract, m_trail, n_trans, n_diff, n_conv, cd, r_eff, front_area, f_rr_c, i_e, i_t, i_ds, ...
         i_diff, i_wheel, b_e, b_t, b_diff] = TractTrailParams(gear);

% gravity
g = 9.81;

% total mass
m_tot = m_tract + m_trail;

%% Calculate Forces
for i = 1:length(Vx)
        
        Fdrive1(i) = -(i_e*n_trans^2*n_diff^2 + (i_t + i_ds)*n_diff^2 + (i_diff + i_wheel))*(Ax(i)/r_eff^2)...
                  + -(b_e*n_trans^2*n_diff^2 + b_t*n_diff^2 + b_diff)*(Vx(i)/r_eff^2) ...
                  + (n_trans*n_diff*torque_e(i)/r_eff);
    
        Fdrive2(i) = n_trans*n_diff*torque_e(i)/r_eff;

%         Fdrive3(i) = torque_e(i)*n_conv*n_trans*n_diff*f_loss/r_eff;
        
        Fdrive3(i) = (torque_e(i) - i_e*Aeng(i))*n_conv*n_trans*n_diff/r_eff;

        Fdamp(i) = (b_e*n_trans^2*n_diff^2 + b_t*n_diff^2 + b_diff)*(Vx(i)/r_eff^2);

        Frot(i) = -(n_trans*n_diff*Aeng(i)*i_e/r_eff);

        Frr(i) = m_tot*g*cos(grade(i))*f_rr_c;
        
        Fgrav(i) = -m_tot*g*sin(grade(i));

        Fdrag = 0;
        
        % sum forces
        Flon1(i) = Fdrive1(i) + Fgrav(i) - Frr(i) - Fdrag;
        Flon2(i) = Fdrive2(i) + Frot(i) - Fdamp(i) - Fgrav(i) - Frr(i) - Fdrag;
        Flon3(i) = Fdrive3(i) - Frr(i) - Fdrag;
    end
end