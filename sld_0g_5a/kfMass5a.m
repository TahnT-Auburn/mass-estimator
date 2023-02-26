%% KF Mass Estimator - Three Axle Tractor
function [gain, covar, innov, m] = kfMass5a(F_lon, grade, g, Ax, phi_init, P_init)
% Author: 
%           Tahn Thawainin, AU GAVLAB
%
% Description: 
%           A function to estimate mass from longitudinal forces using
%           KF
%
% Inputs: 
%           F_lon - longitudinal forces (N)
%           grade - road grade (rad)
%           g - gravity (m/s^2)
%
% Outputs: 
%           m - mass (kg)

%% Initialize

% initial state
phi = phi_init;

% initial covariance
P = P_init;

% state transition matrix
F = 1;

% process noise
Q = 0.01;

% measurement noise
R = 150;

%% RLS
for i = 1:length(F_lon)

    % measurements
    y = F_lon(i);

    % scaling vector
    X = Ax(i) + g*sin(grade(i));

    % time update
    phi = F*phi;    % states
    P = F*P*F' + Q; % covariance matrix
     
    % measurement upate
    S = X*P*X' + R; % innovation covariance 
    K = P*X'/S;     % kalman gain

    % state update
    innov(i) = (y - X*phi);
    phi = phi + K*(innov(i));
    
    % Update covariance matrix
    P = (1 - K*X)*P;

    % Variables
    m(i) = phi; % estimated mass

    covar(:,:,i) = P; % Covariance

    gain(:,i) = K; % Gain

end