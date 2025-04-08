function [next_pose_corrected,next_P] = extended_kalman_filter(current_pose, input, P, Q, R, dt, landmark, sigma)
    % next_pose_predicted = A * current_pose + B * input;
    
    next_pose_predicted = transpose(velocitymodel(current_pose, input, dt));
    A = A_ekf(current_pose,input,dt);
    P_bar = A*P*transpose(A) + Q;
    z_bar = transpose(measurementmodel(next_pose_predicted, landmark));
    z = transpose(sensormeasurement(z_bar(1), z_bar(2), sigma));
    H = H_ekf(next_pose_predicted,landmark);
    K = P_bar * transpose(H) * inv(H*P_bar*transpose(H)+R);
    next_pose_corrected = next_pose_predicted + K * (z-z_bar);
    next_P = (eye(3)-K*H)*P_bar;
end