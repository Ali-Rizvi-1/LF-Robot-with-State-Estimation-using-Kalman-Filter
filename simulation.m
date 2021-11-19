% Sample code 
% (c) Mehdi Raza Khorasani

clc; clear; close all
Steps = 1e2; % number of steps
Points = 1e2; % points to plot

% % System
Ts = 0.2; %sampling time
A = [0 0 1 0;
    0 0 0 1;
    0 0 0 0;
    0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];
sigma_w_2 =1;

syms sw2 T x
F = expm(A*T);
disp(F);
F = double(subs(F,'T',Ts));

%Defining Covariance matrix of noise vector
Qk = int(expm(A*(T-x))*B*...
    [sw2 0;0 sw2]*B'*expm(A*(T-x))',0,T);
disp(Qk);
Qk = subs(Qk,'T',Ts);
Qk = double(subs(Qk,'sw2',sigma_w_2));

% % Sensor
H = eye(4);

%Defining Covariance matrix of noise vector
var_x = 1;
var_y = 1;
var_vx = 1e-1;
var_vy = 1e-1;

Rk = diag([var_x, var_y, var_vx, var_vy]);

% % Kalman Filter Algorithm

% call the function as follows:
xkm1 = zeros(4,1);
xhat_last = zeros(4,1);
P_last = eye(4)*1000;
Error2_xk_yk_Position = 0;
Error2_xk_yk_Velocity = 0;
Error2_xk_xhatk_Position = 0;
Error2_xk_xhatk_Velocity = 0;

for index = 1:Steps
    % Plant noise
    vk=chol(Qk)'*randn(4,1);
    % Plant process model
    xk = F*xkm1 + vk;
    % Measurement noise
    wk=chol(Rk)'*randn(4,1);
    % Measurements
    yk = H*xk + wk;
    % Kalman filter main routine
    [xhat_k,P_k] = KalmanFilter(yk,xhat_last,P_last,F,Qk,H,Rk);
    % Updating last estimates of x_k and P_k
    xkm1 = xk;
    xhat_last = xhat_k;
    P_last = P_k;
    % Computing error metrics
    Error2_xk_yk_Position = ...
        sum((xk(1:2)-yk(1:2)).^2) + Error2_xk_yk_Position;
    Error2_xk_xhatk_Position = ...
        sum((xk(1:2)-xhat_k(1:2)).^2) + Error2_xk_xhatk_Position;
    Error2_xk_yk_Velocity = ...
        sum((xk(3:4)-yk(3:4)).^2) + Error2_xk_yk_Velocity;
    Error2_xk_xhatk_Velocity = ...
        sum((xk(3:4)-xhat_k(3:4)).^2) + Error2_xk_xhatk_Velocity;
    % Saving trajectories
    xtrue(index)=xk(1);
    xmeas(index)=yk(1);
    xestd(index)=xhat_k(1);
    ytrue(index)=xk(2);
    ymeas(index)=yk(2);
    yestd(index)=xhat_k(2);
    vxtrue(index)=xk(3);
    vxmeas(index)=yk(3);
    vxestd(index)=xhat_k(3);
end

Error2_xk_yk_Position = Error2_xk_yk_Position/Steps;
Error2_xk_yk_Velocity = Error2_xk_yk_Velocity/Steps;

Error2_xk_xhatk_Position = Error2_xk_xhatk_Position/Steps;
Error2_xk_xhatk_Velocity = Error2_xk_xhatk_Velocity/Steps;
format short g

disp('MSE for Positions:')
disp(['MSE b/w true and measured states = ' ...
    num2str(Error2_xk_yk_Position)]),
disp(['MSE b/w true and estimated states = ' ...
    num2str(Error2_xk_xhatk_Position)]),
disp('MSE for Velocities:')
disp(['MSE b/w true and measured states = ' ...
    num2str(Error2_xk_yk_Velocity)]),
disp(['MSE b/w true and estimated states = ' ...
    num2str(Error2_xk_xhatk_Velocity)]),

figure;
plot(xtrue(end:-1:end-Points+1),...
    ytrue(end:-1:end-Points+1),'k-'); hold on
plot(xmeas(end:-1:end-Points+1),...
    ymeas(end:-1:end-Points+1),'ro','markersize',5); hold on
plot(xestd(end:-1:end-Points+1),...
    yestd(end:-1:end-Points+1),'b--','linewidth',2)
h=legend('True trajectory', 'Measured trajectory',...
    'Estimated trajectory','location','best');
set(h,'fontsize',16);set(gca,'fontsize',18)

figure;
plot(vxtrue(end:-1:end-Points+1),'k-'); hold on
plot(vxmeas(end:-1:end-Points+1),'ro','markersize',5); hold on
plot(vxestd(end:-1:end-Points+1),'b--','linewidth',2)
h=legend('True velocity', 'Measured velocity',...
    'Estimated velocity','location','best');
set(h,'fontsize',16);set(gca,'fontsize',18)

