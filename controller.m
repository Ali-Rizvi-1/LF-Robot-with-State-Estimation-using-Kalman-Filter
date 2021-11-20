function [delR,delL] = controller(fL,fR,fC)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%[fL,fR,fC]=check_sensors2(RC_Sens(1,:),RC_Sens(2,:));
dd = 20/180*pi; % deg to rad (90 deg/sec)
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
end

