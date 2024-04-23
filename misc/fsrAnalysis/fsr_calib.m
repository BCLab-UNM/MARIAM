%FSR_force sensor calibration
clear all
close all

%data from 100kOhm pull down tests
%averaged voltage sample data
Vout=[0 1.236	2.062	2.958	3.802	4.254	4.644	4.838]; %volts
F = [0 0.04905	0.0981	0.1962	0.4905	0.981	1.962	4.905]; %converted from grams to newtons by dividing by 1000 and multiplying by 9.81

figure
loglog(Vout,F)
xlabel('Measured Voltage [V]')
ylabel('Applied Force to Sensor [N]')
%log-log scale plot we can see this relationship between voltage and force is nonlinear

%take the exponent of the voltage 
x = exp(Vout);
%with basic curve fitting we can get the coefficients of the nonlinear
%relationship with a polynomial with R=1 as:
F_est = 1.60717263158763e-09*x.^5 -3.97326895848675e-07*x.^4 + 3.48267292566989e-05*x.^3 -0.00117407617298212*x.^2 + 0.0234485875465875.*x	-0.0225118029465446;

%plot the actual data vs the estimated forces using the measured voltage
%data
figure
plot((Vout),(F),'linewidth',3)
hold on
plot(Vout,F_est,':','linewidth',3)
xlabel('Measured Voltage [V]')
ylabel('Force [N]')
legend('Data','Estimated')
axis([0 5 0 5])