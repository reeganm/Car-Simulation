%Written by: Balazs Gyenes (2014-03-01)
%Last editted by: Balazs Gyenes (2014-03-01)
%
%DESCRIPTION:
%This script iterates through the findbestratio program to find the minimum
%time to accelerate to 30 km/h for a given range of maximum motor currents.
%The purpose is to see the effect of derating the inverter on our
%acceleration time. To edit simulation parameters, edit the findbestratio
%function.


Imax=50:2.5:80;
MinTime=zeros(1,length(Imax));
IdealRatio=zeros(1,length(Imax));

for i=1:length(Imax)
    [a,b] = findbestratio(0,Imax(i));
    IdealRatio(i)=a;
    MinTime(i)=b;
    
    fprintf('Finished %d iterations of %d.\n',i,length(Imax));
    
end

Problems=(max(MinTime)/max(Imax)^2)*Imax.^2;

figure(1)
plot(Imax,MinTime,Imax,Problems)
legend('Minimum Time','Heat and Losses');
xlabel('Motor Current Limit (A)')
ylabel('Acceleration Time (s)')
title('Minimum Time to Accelerate to 30 km/h vs. Motor Current Limit')
axis([0.9*min(Imax) 1.1*max(Imax) 0 1.2*max(MinTime)]);

figure(2)
plot(Imax,IdealRatio)
xlabel('Motor Current Limit (A)')
ylabel('Ideal Gearing Ratio')
title('Ideal Gearing Ratio vs. Motor Current Limit')
axis([0.9*min(Imax) 1.1*max(Imax) 0 1.2*max(IdealRatio)]);

