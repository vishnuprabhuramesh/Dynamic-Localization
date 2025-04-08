clc; clear; close all;

% Parameters
dt = 0.1;            % Time step
T = 100;             % Total time steps
n_landmarks = 4;

% Initial robot pose [x, y, theta]
x_true = [0; 0; 0];   % True state
x_est = [x_true; reshape(rand(2, n_landmarks)*10, [], 1)];  % Estimate of robot and landmarks

P = eye(length(x_est))*0.1;   % Initial covariance

% Landmark positions and velocities
landmark_pos = rand(2, n_landmarks) * 10;
landmark_vel = randn(2, n_landmarks) * 0.05;

% Obstacle parameters [x, y, radius]
obstacles = [5, 5, 1;
             -2, 3, 1];

% Motion noise and measurement noise
Q = diag([0.01, 0.01, 0.01]);              % Robot motion noise
R = eye(2*n_landmarks) * 0.1;              % Measurement noise

% Store history
history = [];

for t = 1:T
    %% --- Simulate robot control (circular motion) ---
    v = 1.0; w = 0.2;
    u = [v; w];

    % True robot motion
    theta = x_true(3);
    x_true(1:3) = x_true(1:3) + dt * [v*cos(theta); v*sin(theta); w];

    % Update landmark positions
    landmark_pos = landmark_pos + landmark_vel * dt;

    % Generate measurements (relative positions)
    z = [];
    for i = 1:n_landmarks
        lx = landmark_pos(1, i);
        ly = landmark_pos(2, i);
        dx = lx - x_true(1);
        dy = ly - x_true(2);
        z = [z; dx + randn()*sqrt(R(1,1)); dy + randn()*sqrt(R(1,1))];
    end

    %% --- EKF Prediction ---
    theta = x_est(3);
    Fx = [eye(3), zeros(3, 2*n_landmarks)];
    u_hat = [v*cos(theta); v*sin(theta); w];
    x_est(1:3) = x_est(1:3) + dt * u_hat;

    G = eye(length(x_est));
    G(1:3, 1:3) = eye(3) + dt * [0, 0, -v*sin(theta);
                                0, 0,  v*cos(theta);
                                0, 0, 0];
    P = G*P*G' + blkdiag(Q, zeros(2*n_landmarks));

    %% --- EKF Update ---
    z_hat = [];
    H = [];
    for i = 1:n_landmarks
        idx = 3 + 2*i - 1;
        lx = x_est(idx);
        ly = x_est(idx+1);
        dx = lx - x_est(1);
        dy = ly - x_est(2);

        z_hat = [z_hat; dx; dy];

        Hi = zeros(2, length(x_est));
        Hi(:,1:2) = -eye(2);
        Hi(:,idx:idx+1) = eye(2);
        H = [H; Hi];
    end

    S = H*P*H' + R;
    K = P*H' / S;
    y = z - z_hat;
    x_est = x_est + K*y;
    P = (eye(length(x_est)) - K*H)*P;

    %% --- Store and Plot ---
    history = [history, x_est];

    clf;
    hold on; axis equal;
    plot(x_true(1), x_true(2), 'bo', 'MarkerSize', 10, 'DisplayName','Robot True');
    plot(x_est(1), x_est(2), 'kx', 'DisplayName','Robot Est');
    
    % Estimated landmark positions
    for i = 1:n_landmarks
        idx = 3 + 2*i - 1;
        plot(x_est(idx), x_est(idx+1), 'rx', 'MarkerSize', 8);
        plot(landmark_pos(1, i), landmark_pos(2, i), 'g*');
    end

    % Obstacles
    for i = 1:size(obstacles,1)
        %viscircles(obstacles(i,1:2), obstacles(i,3), 'Color','k','LineStyle','--');
        pos = [obstacles(i,1)-obstacles(i,3), obstacles(i,2)-obstacles(i,3), 2*obstacles(i,3), 2*obstacles(i,3)];
        rectangle('Position', pos, 'Curvature', [1 1], 'EdgeColor', 'k', 'LineStyle','--');

    end

    xlim([-5 15]); ylim([-5 15]);
    drawnow;
end
