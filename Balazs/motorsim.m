function[] = motorsim()
%Written by: Balazs Gyenes (2014-02-14)
%Last editted by: Balazs Gyenes (2014-02-15)
%
%DESCRIPTION:
%This script is meant to simulate the performance of a single electric motor
%accelerating as fast as possible, powered by a fuel cell with an
%ultracapacitor bank. At every timestep, the motor wants to apply the
%maximum torque it can, given the constraints of the power train.
%
%HOW TO:
%Scroll to the sections marked Motor Constants, Fuel Cell Parameters, and
%Car Parameters, and input desired values. The motor constants have
%significant impact on drive performance, in conjunction with the gearing
%ratio in the car parameters section. Among the fuel cell parameters, the
%number of cells and the cell area dictate the voltage and current (and
%power) of the fuel cell. Use the FCPolarization script to get a
%polarization curve for your chosen constants. Change the car parameters to
%change the drag conditions of the car. The simulation parameters control
%the step time (keep around 0.05), the stop time, the maximum error (not 
%worth changing), and the option to generate plots or not (if the user only
%wants to know the time to cruising speed).

%MODEL INFORMATION:
%The motor is modeled as a voltage source with value (motor_speed/k_V) in
%series with a resistor representing the winding resistance. The torque
%produced by the motor is proportional to the current through it, less a
%constant idling current. The winding efficiency is the product of voltage
%constant and torque constant, in the proper units. The input motor
%parameters are the voltage constant, winding efficiency, winding
%resistance, maximum current, maximum speed, and idling current.
%
%The fuel cell polarization curve is recreated using physical constants and
%the size of the fuel cell. Use a different script to determine if the
%inputted constants seem reasonable. The input fuel cell parameters are:
%number of cells, cell area, cell membrane resistivity, charge transfer
%coefficient, exchange current density (i_o), cell open circuit voltage,
%diode voltage, and ultracap capacitance.
%
%The drag forces on the car are represented by aerodynamic drag, rolling
%resistance, and a constant drag force ("bearing resistance"). The
%aerodynamic drag goes as 1/2*C_d*A*rho*v^2. The rolling resistance goes as
%C_rr*g*(m+sin(theta)), where theta is the road incline. The input car
%parameters are: mass, wheel radius, height, width, ambient temperature,
%ambient pressure, relative humidity, incline, drag coefficient, rolling resistance
%coefficient, bearing resistance, gearing ratio, and differential
%efficiency.


%MOTOR CONSTANTS
kV=250; %voltage constant, in rpm/Volt
eff_winding=0.90; %winding efficiency
res_winding=80; %winding resistance, in milliOhms
Imax_motor=80; %maximum short term motor current, in A
Vmax_motor=11100; %maximum motor speed, in rpm
I_o=2; %motor idling current, in A

%FUEL CELL PARAMETERS
cellN=50; %number of cells
cellArea=250; %cell area, in cm^2
cellRes=0.62; %cell area resistivity, in Ohm*cm^2
alpha=0.6; %charge transfer coefficient
i_o=0.04; %exchange current density, in mA/cm^2
cellVoc=1.005; %cell open circuit voltage, in V
Vinitial=45; %initial ultracapacitor voltage
Vdiode=0.7; %diode voltage drop at typical operating currents, in V
C=19.3; %ultracapacitor capacitance, in F
cellVtheo=1.48; %cell theoretical open circuit voltage, based on HHV

%CAR PARAMETERS
Mcar=310; %mass of car and driver, in kg
Dwheel=56; %diameter of wheel, in cm
Hcar=1.2; %height of car at widest point, in m
Wcar=1.67; %width of car at widest point, in m
Temp=30; %ambient temperature, in degrees C
Pressure=101; %ambient pressure, in kPa
RH=50; %relative humidity, in %
incline=0; %road incline, in degrees
C_d=0.3; %aerodynamic drag coefficient
C_rr=0.01; %coefficient of rolling resistance
C_br=10; %constant drag force ('bearing resistance'), in N
k_gear=28; %gearing ratio
eff_diff=90; %differential efficiency, in %

%SIMULATION PARAMETERS
tstep=0.0001; %time step, in seconds
tstop=120; %stop time, in seconds
errormax=0.01; %maximum error in node currents, in A
makeplots=1; %should the script generate plots?



%SCRIPT:

%Motor Calculations
kT=eff_winding/(kV*2*pi/60);
res_winding=res_winding/1000;

%Fuel Cell Calculations
Imax_FC=0.35*cellArea;
A=8.314*(273.15+Temp)/2/alpha/96485;
%gainVlimit=-(cellRes/cellArea*cellN/res_winding+cellRes/cellArea*cellN*C/tstep+1);
%gainClimit=-(cellRes/cellArea*cellN*C/tstep+1)

%Car Calculations
Rwheel=Dwheel/2/100;
density=((Pressure-(RH/100)*exp(-42800/8.314462*(1/(Temp+273.15)-1/373.15)+log(101.325)))*28.9644+(RH/100)*exp(-42800/8.314462*(1/(Temp+273.15)-1/373.15)+log(101.325))*18.02)/(8.314462*(Temp+273.15));
eff_diff=eff_diff/100;


time=0:tstep:tstop;
Npoints=length(time);

Speed=zeros(1,Npoints);
SpeedKMPH=zeros(1,Npoints);
MotorSpeed=zeros(1,Npoints);
DragForce=zeros(1,Npoints);
FCCurrent=zeros(1,Npoints);
FCVoltage=zeros(1,Npoints);
CapCurrent=zeros(1,Npoints);
MotorCurrent=zeros(1,Npoints);
MotorTorque=zeros(1,Npoints);
Acceleration=zeros(1,Npoints);
PowerIn=zeros(1,Npoints);
PowerOut=zeros(1,Npoints);
FCEfficiency=zeros(1,Npoints);
MotorEfficiency=zeros(1,Npoints);


Speed(1)=0;
SpeedKMPH(1)=Speed(1)*3.6;
MotorSpeed(1)=Speed(1)/Rwheel/2/pi*60*k_gear;
DragForce(1)=0.5*C_d*Hcar*Wcar*density*Speed(1)^2+(C_rr+sind(incline))*Mcar*9.81+C_br;

%fuel cell current according to max fuel cell current
IFClimit=Imax_FC;

%fuel cell current according to max motor current
I=0;
error=errormax+1;
while (abs(error)>errormax)
    
    error=(getVoltage(I,cellN,cellArea,[cellRes alpha i_o cellVoc])-Vdiode)*(C/tstep)-I+Imax_motor-C*Vinitial/tstep;
    gain=-((cellRes/cellArea*cellN+min(1000,A*cellN/I))*C/tstep+1);
    I=max(0,I-error/gain);
    
end
IMlimit=I;

%fuel cell current according to fuel cell voltage limit
I=0;
error=errormax+1;
while (abs(error)>errormax)
    
    error=(getVoltage(I,cellN,cellArea,[cellRes alpha i_o cellVoc])-Vdiode)*(1/res_winding+C/tstep)-I-MotorSpeed(1)/kV/res_winding-C*Vinitial/tstep;
    gain=-((cellRes/cellArea*cellN+min(1000,A*cellN/I))*(1/res_winding+C/tstep)+1);
    I=max(0,I-error/gain);
    
end
IVlimit=I;

FCCurrent(1)=min([IFClimit,IMlimit,IVlimit]);
FCVoltage(1)=getVoltage(FCCurrent(1),cellN,cellArea,[cellRes alpha i_o cellVoc]);
CapCurrent(1)=C*(Vinitial-(FCVoltage(1)-Vdiode))/tstep;
MotorCurrent(1)=CapCurrent(1)+FCCurrent(1);
MotorTorque(1)=kT*(MotorCurrent(1)-I_o);
Acceleration(1)=(MotorTorque(1)*k_gear*eff_diff/Rwheel-DragForce(1))/Mcar;
PowerIn(1)=FCCurrent(1)*FCVoltage(1);
PowerOut(1)=MotorTorque(1)*eff_diff*MotorSpeed(1)*2*pi/60;
FCEfficiency(1)=FCVoltage(1)/(cellN*cellVtheo);
MotorEfficiency(1)=MotorTorque(1)*MotorSpeed(1)*2*pi/60/((FCVoltage(1)-Vdiode)*MotorCurrent(1));


for i=2:Npoints
    
    Speed(i)=Speed(i-1)+Acceleration(i-1)*tstep;
    SpeedKMPH(i)=Speed(i)*3.6;
    MotorSpeed(i)=Speed(i)/Rwheel/2/pi*60*k_gear;
    DragForce(i)=0.5*C_d*Hcar*Wcar*density*Speed(i)^2+(C_rr+sind(incline))*Mcar*9.81+C_br;
    
    if(MotorSpeed(i)<Vmax_motor)
        
        %fuel cell current according to max fuel cell current
        IFClimit=Imax_FC;
        
        %fuel cell current according to max motor current
        I=FCCurrent(i-1);
        error=errormax+1;
        while (abs(error)>errormax)
            
            error=(getVoltage(I,cellN,cellArea,[cellRes alpha i_o cellVoc])-Vdiode)*(C/tstep)-I+Imax_motor-C*(FCVoltage(i-1)-Vdiode)/tstep;
            gain=-((cellRes/cellArea*cellN+min(1000,A*cellN/I))*C/tstep+1);
            I=max(0,I-error/gain);
            
        end
        IMlimit=I;
        
        %fuel cell current according to fuel cell voltage limit
        I=FCCurrent(i-1);
        error=errormax+1;
        while (abs(error)>errormax)
            
            error=(getVoltage(I,cellN,cellArea,[cellRes alpha i_o cellVoc])-Vdiode)*(1/res_winding+C/tstep)-I-MotorSpeed(i)/kV/res_winding-C*(FCVoltage(i-1)-Vdiode)/tstep;
            gain=-((cellRes/cellArea*cellN+min(1000,A*cellN/I))*(1/res_winding+C/tstep)+1);
            I=max(0,I-error/gain);
            
        end
        IVlimit=I;
        
        FCCurrent(i)=min([IFClimit,IMlimit,IVlimit]);
        
    else
        
        Imax=DragForce(i)*Rwheel/k_gear/eff_diff/kT+I_o;
        I=FCCurrent(i-1);
        error=errormax+1;
        while (abs(error)>errormax)
            
            error=(getVoltage(I,cellN,cellArea,[cellRes alpha i_o cellVoc])-Vdiode)*(C/tstep)-I+Imax-C*(FCVoltage(i-1)-Vdiode)/tstep;
            gain=-((cellRes/cellArea*cellN+min(1000,A*cellN/I))*C/tstep+1);
            I=max(0,I-error/gain);
            
        end
        
        FCCurrent(i)=I;
        
    end
    
    FCVoltage(i)=getVoltage(FCCurrent(i),cellN,cellArea,[cellRes alpha i_o cellVoc]);
    CapCurrent(i)=C*((FCVoltage(i-1)-Vdiode)-(FCVoltage(i)-Vdiode))/tstep;
    MotorCurrent(i)=CapCurrent(i)+FCCurrent(i);
    MotorTorque(i)=kT*(MotorCurrent(i)-I_o);
    Acceleration(i)=(MotorTorque(i)*k_gear*eff_diff/Rwheel-DragForce(i))/Mcar;
    PowerIn(i)=FCCurrent(i)*FCVoltage(i);
    PowerOut(i)=MotorTorque(i)*eff_diff*MotorSpeed(i)*2*pi/60;
    FCEfficiency(i)=FCVoltage(i)/(cellN*cellVtheo);
    MotorEfficiency(i)=MotorTorque(i)*MotorSpeed(i)*2*pi/60/((MotorSpeed(i)/kV+MotorCurrent(i)*res_winding)*MotorCurrent(i));
    
end

if makeplots
    
    figure(1)
    plot(time,SpeedKMPH,'LineWidth',2);
    axis([0 tstop 0 50]);
    title('Speed Profile');
    xlabel('Time (s)');
    ylabel('Vehicle Speed (km/h)');
    
    figure(2)
    plot(time,Acceleration,'LineWidth',2);
    title('Acceleration Profile');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    
    figure(3)
    plot(time,FCCurrent,time,MotorCurrent,time,CapCurrent);
    legend('Stack Current','Motor Current','Ultracap Current');
    title('System Currents');
    xlabel('Time (s)');
    ylabel('Current (A)');
    
    figure(4)
    plot(time,FCVoltage);
    axis([0 tstop 0 50]);
    title('Fuel Cell Voltage');
    xlabel('Time (s)');
    ylabel('Voltage (V)');
    
    figure(5)
    plot(time,PowerIn,time,PowerOut);
    legend('Fuel Cell Power','Power at Wheels');
    title('Drivetrain Power');
    xlabel('Time (s)');
    ylabel('Power (W)');
    
    figure(6)
    plot(time,MotorTorque);
    axis([0 tstop 0 max(MotorTorque)*1.1]);
    title('Motor Torque');
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    
    figure(7)
    plot(time,MotorSpeed);
    axis([0 tstop 0 Vmax_motor*1.1]);
    title('Motor Speed');
    xlabel('Time (s)');
    ylabel('Speed (rpm)');
    
    figure(8)
    plot(time,FCEfficiency,time,MotorEfficiency);
    legend('Fuel Cell Efficiency','Motor Efficiency');
    axis([0 tstop 0 1]);
    title('Drivetrain Efficiencies');
    xlabel('Time (s)');
    ylabel('Efficiency');
    
end


tcruise=time(find(SpeedKMPH>30,1,'first'));

fprintf('It takes %4.1f seconds for the car to accelerate to 30 km/h.\n',tcruise);


end


function[Voltage]=getVoltage(Current,NCells,CellArea,ParametersArray)
%Written by: Balazs Gyenes (2014-02-15)
%Last editted by: Balazs Gyenes (2014-02-15)

%This function takes fuel cell physical parameters and an input current
%(in Amps) and returns the fuel cell voltage that corresponds to that current.

%The ParametersArray input takes the following form:
%[cell_area_resistivity (Ohm*cm^2) (typ. 0.24 Ohm*cm^2),
%charge_transfer_coefficient (unitless) (typ. 0.6),
%i_o (mA/cm^2) (typ. 0.04 mA/cm^2),
%cell_OC_voltage (V) (typ. 1.03) ]

T=300; %temp, in K
F=96485; %Faraday constant, in C/mol
R=8.314462; %Real Gas Constant, in J/molK

CellRes=ParametersArray(1);
alpha=ParametersArray(2);
i_o=ParametersArray(3);
CellV_OC=ParametersArray(4);

A=R*T/2/alpha/F; %Tafel coefficient, in V

Voltage=NCells*(CellV_OC-Current*CellRes/CellArea-max(0,A*log(Current/(CellArea*i_o/1000))));


end