function [xhat_k, P_k] ...
    = KalmanFilter(y_k, xhat_last, P_last, F, Qk, H, Rk)
%The Kalman Filter Algorithm: 
% Inputs (In order):
% yk: measurement at time k 
% xhat_last_given_last: Optimal estimate at time k - 1
% P_last_given_last: Covariance matrix of X at time k-1
% F, Q: System and its noise Covariance matrix
% H, R: Sensor and its noise Covariance matrix
%
%Outputs: 
% xhat_k_given_k: Optimal estimate at time k
% P_k_given_k: Variance of X at time k 
% (c) Mehdi Khorasani

% State Prediction
[xhat_predict, P_predict] = ...
    state_predict(xhat_last ,P_last, F, Qk); 
% Measurement Prediction
[yhat_predict, S_k] = ...
    measurement_predict(xhat_predict,P_predict, H, Rk); 
% Kalman Correction Step
[xhat_k,P_k] = ...
    correction_step(S_k,yhat_predict,y_k,H,xhat_predict,P_predict);

end