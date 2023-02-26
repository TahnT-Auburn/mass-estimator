%% Straight Line Drive with 0 grade data analysis - Three Axle Tractor
% Tahn Thawainin

clc;
clear variables;
close all;

%% Load data

data = load('Run106.mat');

%%  Variables

% set gear
gear = 10;

% vehicle parameters
[m_tract, n_trans, n_diff, n_conv, cd, r_eff, front_area, f_rr_c, i_e, i_t, i_ds, ...
         i_diff, i_wheel, b_e, b_t, b_diff] = TractParams(gear);

% gravity
g = 9.81;

% tractor longitudinal velocity (km/h)
Vx = data.Vx;
% tractor longitudinal velocity (m/s)
Vx = Vx*(1e3/3600);

% tractor longitudinal acceleration (g's)
Ax = data.Ax;
% tractor longitudinal acceleration (m/s^2)
Ax = g*Ax;

% engine torque (N-m)
torque_e = data.M_EngOut;

% engine accel (rad/s^2)
Aeng = data.AA_Eng;

% set grade (zero grade case)
grade = zeros(1,length(Vx));

% time
T = data.T_Event;

%% Truth mass

% true mass
m_truth = m_tract*ones(1,length(Vx));

% 2% error bounds
m_upper_error = m_tract + 0.02*m_tract;
m_lower_error = m_tract - 0.02*m_tract;

m_upper_bound = m_upper_error*ones(1,length(Vx));
m_lower_bound = m_lower_error*ones(1,length(Vx));

%% Longitudinal Forces

% call function
[Flon1, Flon2, Flon3] = lonForces(Ax, Aeng, Vx, torque_e, gear, grade);

%% Initialize Estimators

% initial mass estimate
phi_init = 5000;

% intial covariance
P_init  = 5500;

%% LS Mass Estimator

[gain_ls, covar_ls, innov_ls, m_ls] = rlsMass3a(Flon3, grade, g, Ax, phi_init, P_init);

% plot
figure
hold on
plot(T, m_truth, DisplayName='Truth')
plot(T, m_ls, DisplayName='LS');
plot(T, m_upper_bound, '--', color = 'r', DisplayName='2% upper bound')
plot(T, m_lower_bound, '--', color = 'r', DisplayName='2% lower bound')
hold off
title('LS Mass Estimator')
xlabel('Time [s]')
ylim([0, 2e4])
ylabel('kg')
legend(Location='best')
grid

%% KF Mass Estimator

[gain_kf, covar_kf, innov_kf, m_kf] = kfMass3a(Flon3, grade, g, Ax, phi_init, P_init);

% plot
figure
hold on
plot(T, m_truth, DisplayName='Truth')
plot(T, m_kf, DisplayName='KF')
plot(T, m_upper_bound, '--', color = 'r', DisplayName='2% upper bound')
plot(T, m_lower_bound, '--', color = 'r', DisplayName='2% lower bound')
hold off
title('KF Mass Estimator')
xlabel('Time [s]')
ylabel('kg')
legend(Location='best')
grid

