function [next_pose_corrected,next_P] = kalman_filter(current_pose, input, P, Q, R, A, B, H, z)
    next_pose_predicted = A * current_pose + B * input;
    P_bar = A*P*transpose(A) + Q;
    z_bar = H*next_pose_predicted;
    K = P_bar * transpose(H) * inv(H*P_bar*transpose(H)+R);
    next_pose_corrected = next_pose_predicted + K * (z-z_bar);
    next_P = (eye(3)-K*H)*P_bar;
end