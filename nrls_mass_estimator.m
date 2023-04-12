%% Nonlinear Recursive Least Squares Mass Estimation
% Tahn Thawainin

clc
clear variables
close all

%% Load Data 

% simulation data set
data  = load("l5a_p_sg_ct2.mat");

% vehicle parameters
vp = data.vp;
% procedure
pro = data.pro;
% simulation
sim = data.sim;

% Extract Simulation Variables
% position
pos = extractfield(sim,'pos');

% velocity
vel = extractfield(sim,'vel');

% acceleration
accel = extractfield(sim,'accel');

% engine torque
T_eng = extractfield(sim, 'T_eng');

% drag force
F_drag = extractfield(sim, 'F_drag');

% rolling resistance
F_rr = extractfield(sim, 'F_rr');

% grade force
F_grade = extractfield(sim, 'F_grade');

%% Measurements

% measurement variance
sigma_v = 0.1;

% measurement
y = accel + sigma_v*randn(1,length(pro.t_sim));

%% Nonlinear Recursive Least Squares

% intialize
x_init = 1/7000;
P_init = 1e-8;

x = x_init;
P = P_init;

% measurement 
R = 1e-5;

for k = 1:length(pro.t_sim)
    
    % linearized observation matrix
    H = pro.scale_factor*T_eng(k) - pro.B_eff*vel(k)...
        - vp.u_rr*pro.g*cos(pro.grade(k)) - pro.g*sin(pro.grade(k)) ...
        - 0.5*pro.p*vp.cd*vp.front_area*vel(k)^2;

    % nonlinear observation matrix
    h = pro.scale_factor*T_eng(k) - pro.B_eff*vel(k)...
        - ((1/x) - pro.M_i)*vp.u_rr*pro.g*cos(pro.grade(k)) ...
        - ((1/x) - pro.M_i)*pro.g*sin(pro.grade(k)) ...
        - 0.5*pro.p*vp.cd*vp.front_area*vel(k)^2;
    
    % measurement noise
    if abs(y(k)) > 0.05
        R = 1e-5;
    elseif abs(y(k)) < 0.05
        R = 1;
    end
    
    % gain
    L = P*H'/(H*P*H' + R);

    % covariance
    P = (eye(1) - L*H)*P;

    % state estimate
    x = x + L*(y(k) - h*x);

    % siphon variables

    % effective mass estimate
    M_eff_est(k) = (1/x);
    % vehicle mass estimate
    M_veh_est(k) = (1/x) - pro.M_i;

    % kalman gain
    gain(k) = L;
    % covariance matrix
    covar(k) = P;
    % measurement variance
    meas_var(k) = R;
end

%% Interface

% display vehicle info
veh_info = 'true';
% display procedure info
pro_info = 'true';
% display simulation info
sim_info = 'false';
% display estimator info
est_info = 'true';

% vehicle info
if strcmp(veh_info, 'true') == 1
    
    disp('Vehicle Configuration:')

    if vp.config == 0

        disp('3 axle tractor')

        disp('vehicle mass')
        disp(vp.m_veh)

    elseif vp.config == 1

        disp('5 axle unloaded tractor + trailer')
    
        disp('vehicle mass')
        disp(vp.m_veh)

    elseif vp.config == 2

        disp('5 axle loaded tractor + trailer')

        disp('load mass')
        disp(vp.m_l)

        disp('vehicle mass')
        disp(vp.m_veh)
    end

elseif strcmp(veh_info, 'false') == 1
end

% procedure info
if strcmp(pro_info, 'true') == 1

    disp('Procedure Specs:')
    
    disp('sampling rate')
    disp(pro.dt)

    disp('simulation time')
    disp(pro.t_run)
    
    disp('initial position')
    disp(pro.pos_init)

    disp('initial velocity')
    disp(pro.v_init)
    
    disp('desired velocity')
    disp(pro.v_des)
    
    if pro.grade_status == 0
    
    disp('grade status: constant')
   
    disp('grade (deg)')
    disp(pro.grade_deg)

    elseif pro.grade_status == 1
    
    disp('grade status: sinusoidal')

    disp('grade mag (deg)')
    disp(pro.grade_deg)
    
    disp('grade change frequecy')
    disp(pro.grade_freq)
    end

elseif strcmp(pro_info, 'false') == 1
end

% simulation information
if strcmp(sim_info, 'true') == 1
    
    % sim specs
    figure
    set(gcf,'color','w')
    subplot(4,1,1)
    plot(pro.t_sim, pos)
    title('Position')
    ylabel('m')
    grid

    subplot(4,1,2)
    plot(pro.t_sim, vel)
    title('Velocity')
    ylabel('m/s')
    grid

    subplot(4,1,3)
    plot(pro.t_sim, accel)
    title('Acceleration')
    ylabel('m/s^2')
    grid

    subplot(4,1,4)
    plot(pro.t_sim, T_eng)
    title('Engine Torque')
    ylabel('N-m')
    xlabel('Time [s]')
    grid
    
    % forces
    figure
    set(gcf,'color','w')
    subplot(3,1,1)
    plot(pro.t_sim, F_drag)
    title('Drag Force')
    ylabel('N')
    grid

    subplot(3,1,2)
    plot(pro.t_sim, F_rr)
    title('Rolling Force')
    ylabel('N')
    grid

    subplot(3,1,3)
    plot(pro.t_sim, F_grade)
    title('Grade Force')
    ylabel('N')
    xlabel('Time [s]')
    grid
elseif strcmp(sim_info, 'false') == 1
end

% estimator information
if strcmp(est_info, 'true') == 1
    
    % estimator specs
    disp('Estimator Specs:')

    disp('Measurement Noise:')
    disp(R)

    % NRLS comparison plot
    figure
    set(gcf,'color','w')
    hold on
    plot(pro.t_sim, vp.m_veh*ones(1,length(pro.t_sim)), '--', LineWidth = 1.5)
    plot(pro.t_sim, M_veh_est, LineWidth = 1.5)
    hold off
    title('Vehicle Mass Estimate')
    xlabel('Time [s]')
    ylabel('Kg')
    ylim([(vp.m_veh - 0.05*vp.m_veh),(vp.m_veh + 0.05*vp.m_veh)])
    grid
    legend('Truth', 'NRLS')

elseif strcmp(est_info, 'false') == 1
end