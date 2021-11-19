function animation_LMS
% Goal: the purpose of this program is to
% keep the robot follow an elliptic contour.

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

dd = 20/180*pi; % deg to rad (90 deg/sec)
dev_w = 1e-2;
dev_v = 1e-2;
trajectory =zeros(2,N);
RC_Sens = [cos(phi) -sin(phi); ...
    sin(phi) cos(phi)]*[Sen_X; Sen_Y]+[x_pos;y_pos];

for ii = 1:N
    [fL,fR,fC]=check_sensors2(RC_Sens(1,:),RC_Sens(2,:));
    if fL == 1 && fR == 0 && fC == 0
        delR = dd*2;
        delL = -dd;
    elseif fL == 0 && fR == 1 && fC == 0
        delR = -dd;
        delL = dd*2;
    elseif fL == 1 && fR == 1 && fC == 0
        delR = dd*4;
        delL = -dd*4;
    else
        delR = dd*4;
        delL = dd*4;
    end
    
    vee = rw*(delR+delL)/2 +randn*dev_v; % noise added
    omega = rw*(delR-delL)/2/b +randn*dev_w; % noise added
    
    delta_phi = T*omega;
    delta_x = 2*vee/omega*sin(delta_phi/2)*cos(phi+delta_phi/2);
    delta_y = 2*vee/omega*sin(delta_phi/2)*sin(phi+delta_phi/2);
    
    x_pos = x_pos + delta_x;
    y_pos = y_pos + delta_y;
    phi = wrapToPi(phi + delta_phi);
    
    RC_Sens = [cos(phi) -sin(phi); ...
        sin(phi) cos(phi)]*[Sen_X; Sen_Y]+[x_pos;y_pos];
    RC_Body = [cos(phi) -sin(phi); ...
        sin(phi) cos(phi)]*[Body_X;Body_Y]+[x_pos;y_pos];
    
    trajectory(:,ii) =[RC_Sens(1,3); RC_Sens(2,3)];
    clf;plot(ellipse_a*(r-s*q)*cos(theta),...
        ellipse_b*(r-s*q)*sin(theta),'k-');hold on
    plot(ellipse_a*(r+s*q)*cos(theta),...
        ellipse_b*(r+s*q)*sin(theta),'k-');hold on
    patch(RC_Body(1,:),RC_Body(2,:),'g'); alpha(0.2);
    plot(RC_Sens(1,:),RC_Sens(2,:),...
        'o','markersize',2,'markerfacecolor','k')
    axis([-1 1 -1 1]*bb);hold on
    plot(trajectory(1,1:ii),trajectory(2,1:ii),'b-','linewidth',2)
    axis square; drawnow;
end

function [fL,fR,fC]=check_sensors2(sen_x,sen_y)
global r q s ellipse_a ellipse_b
fL=0;fR=0;fC=0;
ai = ellipse_a*(r-s*q);
bi = ellipse_b*(r-s*q);
ao = ellipse_a*(r+s*q);
bo = ellipse_b*(r+s*q);
x=sen_x(1);
y=sen_y(1);
if x^2/ai^2+y^2/bi^2>1 && x^2/ao^2+y^2/bo^2<1
    fL = 1;
end
x=sen_x(2);
y=sen_y(2);
if x^2/ai^2+y^2/bi^2>1 && x^2/ao^2+y^2/bo^2<1
    fR = 1;
end
x=sen_x(3);
y=sen_y(3);
if x^2/ai^2+y^2/bi^2>1 && x^2/ao^2+y^2/bo^2<1
    fC = 1;
end