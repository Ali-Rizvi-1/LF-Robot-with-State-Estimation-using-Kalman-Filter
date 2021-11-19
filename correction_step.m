function [xhat_k,P_k] = correction_step(S_k,yhat_predict,...
    y_k,H,xhat_predict,P_predict)

K_k = (P_predict* H')*inv(S_k); 
xhat_k = xhat_predict + K_k*(y_k - yhat_predict); 
P_k = P_predict - K_k*H*P_predict; 
