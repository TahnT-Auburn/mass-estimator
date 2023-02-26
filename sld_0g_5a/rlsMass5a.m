%% LS/RLS Mass Estimator - Three Axle Tractor
function [gain, covar, innov, m] = rlsMass5a(F_lon, grade, g, Ax, phi_init, P_init)
% Author: 
%           Tahn Thawainin, AU GAVLAB
%
% Description: 
%           A function to estimate mass from longitudinal forces using
%           LS/RLS
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

%% RLS
for i = 1:length(F_lon)

    % measurements
    y = F_lon(i);

    % scaling vector
    X = Ax(i) + g*sin(grade(i));

%     % update gain matrix
%     L = P*X'./(1 + X*P*X');
% 
%     % update states
%     innov(i) = (y - X*phi);
%     phi = phi + L*(innov(i));
% 
%     % Update covariance matrix
%     P = (1 - L*X)*P;

    % direct least squares
    innov(i) = 1;
    while abs(innov(i)) > 0.1

    % update gain matrix
    L = P*X'./(1 + X*P*X');

    % update states
    innov(i) = (y - X*phi);
    phi = phi + pinv(X)*innov(i);
    
    % Update covariance matrix
    P = (1 - L*X)*P;

    end

    % Variables
    m(i) = phi; % estimated mass

    covar(:,:,i) = P; % Covariance

    gain(:,i) = L; % Gain

end