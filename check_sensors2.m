function [fL,fR,fC]=check_sensors2(sen_x,sen_y)
%check_sensors2 Summary of this function goes here
%   Detailed explanation goes here
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
