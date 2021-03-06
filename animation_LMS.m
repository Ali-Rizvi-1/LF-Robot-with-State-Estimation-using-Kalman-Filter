function animation_LMS
% Goal: the purpose of this program is to
% keep the robot follow an elliptic contour. 
% implementing with Extended Kalman Filter
close all; clear; clc;

global s d b r bb q ellipse_a ellipse_b

N = 1e3; % number of time steps
d = 2; % the platform length of the robot
b = 2; % 2b is the platform width of the robot
s = b/3; % s, d1 and d2 determine positions of the sensor
d1 = d/3;
d2 = d/2;
r = 5*d; % radius of the path
rw = 1/2; % Wheel radius
q = 0.85; % the ratio of track-width and distance b/w wheels
T = 0.3; % Sampling time
ellipse_a = 1;
ellipse_b = 1.2;
bb = (ellipse_b+ellipse_a/2)*r;

% Body base position in global frame of reference
Body = [-b b -d d];
Body_X = [Body(3) Body(3) Body(4) Body(4)];
Body_Y = [Body(1) Body(2) Body(2) Body(1)];

% Sensors base position in global frame of reference
Sen_X = [-d1 -d1 d2];
Sen_Y = [s -s 0];
theta=linspace(0,2*pi,100); % length is fixed

% Robot is placed at (x,y) = (r,0)
x_pos = rand*10;
y_pos = rand*10;
phi = (2*rand-1)*pi;

%KF storage variables: 
xhat_last = [x_pos; y_pos; phi; 0; 0];

%stochastic setup:
kR = 1e-2; kL = 1e-2;
p_last = eye(5);

dd = 20/180*pi; % deg to rad (90 deg/sec)
dev_w = 1e-2;
dev_v = 1e-2;
KF_trajectory = zeros(2,N);
trajectory =zeros(2,N);
RC_Sens = [cos(phi) -sin(phi); ...
    sin(phi) cos(phi)]*[Sen_X; Sen_Y]+[x_pos;y_pos];

%stochastic setup for observation: 
R_k = diag([1e-3 1e-3]);

aaa=figure('name', 'Odin')
hAx1 = axes;
%plot(hAx1, 1, 1, '*r')
hold on

bbb=figure('name', 'Thor')
hAx2 = axes;
%plot(hAx2, 2, 1, '*r')
hold on

for ii = 1:N
    [fL,fR,fC]=check_sensors2(RC_Sens(1,:),RC_Sens(2,:));
    
    [delR,delL] = controller(fL,fR,fC);
    %xhat_last = [x_pos; y_pos; phi; delR; delL];
    xhat_last(4) = delR;
    xhat_last(5) = delL;
    
    %calculate input's Covariance matrix: 
    Q_input = diag([kR*abs(delR) kL*abs(delL)]);
    
    vee = rw*(delR+delL)/2 +randn*sqrt(Q_input(1,1)); % noise added
    omega = rw*(delR-delL)/2/b +randn*sqrt(Q_input(2,2)); % noise added
    
    delta_phi = T*omega;
    delta_x = 2*vee/omega*sin(delta_phi/2)*cos(phi+delta_phi/2);
    delta_y = 2*vee/omega*sin(delta_phi/2)*sin(phi+delta_phi/2);
     
    x_pos = x_pos + delta_x;
    y_pos = y_pos + delta_y;
    phi = wrapToPi(phi + delta_phi);
    
    %Propagating input error covariance:
    sigma_ = diag([kR*delR kL*delL]);
    B = [(1/2)*cos(phi+T*omega/2)-(T*vee/(2*b))*sin(phi+T*omega/2), (1/2)*cos(phi+T*omega/2)+(T*vee/(2*b)*sin(phi+T*omega/2));
        (1/2)*sin(phi+T*omega/2)+(T*vee/(2*b))*cos(phi+T*omega/2), (1/2)*sin(phi+T*omega/2)-(T*vee/(2*b))*cos(phi+T*omega/2);
        1/b, -1/b;
        1,0;
        0,1];
    Q_k = B*sigma_*B';
    
    %observation: Assume radar observation 
    
    y_k = [delR 0;0 delL] + sqrt(R_k)*randn(2, 1); 
    
    %y_k = [sqrt(x_pos^2+y_pos^2); atan(y_pos/x_pos)] + sqrt(R_k)*randn(2, 1);
    
    %compute the jacobian of f at x(k-1 given k-1)
    F_k = [1 0 -T*vee*sin(phi+(T*omega)/2); 0 1 T*vee*cos(phi+(T*omega)/2); 0 0 1];
    
    F_k = [F_k [0;0;0] [0;0;0]; 0 0 0 0 0; 0 0 0 0 0];
    
    % H jacobian 
    %rho = sqrt(x_pos^2+y_pos^2); 

    %H_k = [x_pos/rho y_pos/rho 0; -y_pos/rho^2 x_pos/rho^2 0];
    
    H_k = [0 0 0 1 0;
        0 0 0 0 1];
    
    %KF:
    %[xhat_k, P_k] = KalmanFilter(y_k, xhat_last, P_last, F, Qk, H, Rk)
    [xhat_optimal,P_optimal] = KalmanFilter(y_k, xhat_last, p_last, F_k, Q_k, H_k, R_k);
    
    xhat_last = xhat_optimal; 
    p_last = P_optimal;
    
    RC_Sens = [cos(phi) -sin(phi); ...
        sin(phi) cos(phi)]*[Sen_X; Sen_Y]+[x_pos;y_pos];
    RC_Body = [cos(phi) -sin(phi); ...
        sin(phi) cos(phi)]*[Body_X;Body_Y]+[x_pos;y_pos];
    
    trajectory(:,ii) =[RC_Sens(1,3); RC_Sens(2,3)];
    
    %Transformation matrices: Filtered
    RC_Sens_KF = [cos(xhat_optimal(3)) -sin(xhat_optimal(3)); ...
        sin(xhat_optimal(3)) cos(xhat_optimal(3))]*[Sen_X; Sen_Y]+[xhat_optimal(1);xhat_optimal(2)]; 
    
    RC_Body_KF = [cos(xhat_optimal(3)) -sin(xhat_optimal(3)); ...
        sin(xhat_optimal(3)) cos(xhat_optimal(3))]*[Body_X;Body_Y]+[xhat_optimal(1);xhat_optimal(2)];

    KF_trajectory(:, ii) = [RC_Sens_KF(1,3); RC_Sens_KF(2,3)];
    
%     hold on 
%     plot(x_pos,y_pos); 
%     plot(xhat_optimal(1),xhat_optimal(2));
%     hold off
%     legend('Ground Truth','Estimated');

    plot(hAx1,x_pos,...
        y_pos,'k*'); hold on
    plot(hAx1,xhat_optimal(1),...
        xhat_optimal(2),'ro','markersize',5);
    
    %clf(bbb);
    plot(hAx2,ellipse_a*(r-s*q)*cos(theta),...
        ellipse_b*(r-s*q)*sin(theta),'k-');hold on
    plot(hAx2,ellipse_a*(r+s*q)*cos(theta),...
        ellipse_b*(r+s*q)*sin(theta),'k-');hold on
    
    patch(hAx2, RC_Body(1,:),RC_Body(2,:),'g'); alpha(0.2);
    plot(hAx2,RC_Sens(1,:),RC_Sens(2,:),...
        'o','markersize',2,'markerfacecolor','k')
    
    patch(hAx2,RC_Body_KF(1,:),RC_Body_KF(2,:),'g'); alpha(0.2);
    plot(hAx2,RC_Sens_KF(1,:),RC_Sens_KF(2,:),...
        'o','markersize',2,'markerfacecolor','k')
    
    axis([-1 1 -1 1]*bb);hold on
    plot(hAx2,trajectory(1,1:ii),trajectory(2,1:ii),'b-','linewidth',2)
    plot(hAx2,KF_trajectory(1,1:ii),KF_trajectory(2,1:ii),'r-','linewidth',2)
    axis square; drawnow;
    
end
