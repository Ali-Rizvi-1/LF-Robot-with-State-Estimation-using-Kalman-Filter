function [yhat_predict, S_k] ...
    = measurement_predict(xhat_predict,P_predict, H, Rk)
%Calculates the measurement's prediction in KF algorithm

yhat_predict = H*xhat_predict; 
S_k = H*P_predict * H' + Rk; 
end