%FSR_force sensor calibration
clear all
close all

% Read data from table
% Specify the path to the CSV file
filename = 'fsrCalibrationAverages1.csv';
TData = readtable(filename);

% Get data meta-data
variableNames = TData.Properties.VariableNames(2:end);
totalResistors = size(variableNames, 2);
totalForces = size(variableNames, 1);
colors = cool(totalResistors);

% Observed data from 100kOhm pull down tests, averaged voltage sample TData
% Converted from grams to newtons by dividing by 1000 and multiplying by 9.81
% vOut_Jon =[0 1.236	2.062	2.958	3.802	4.254	4.644	4.838];
% force_Jon = [0 0.04905	0.0981	0.1962	0.4905	0.981	1.962	4.905]; 

% Assign voltageOut and Force data from file
vOut = TData.x100k;
voltageOut = TData(:, 2:end)
force = (TData.Resistors / 1000)*9.81

% Log-log scale plot we can see this relationship between voltage and force is nonlinear
legendTitles = {};
figure
for i = 1:length(variableNames)
    varName = variableNames{i};
    loglog(voltageOut.(varName), force, 'LineWidth', (0.5*i)+1, 'Color', colors(i, :));
    legendTitles{end+1} = ['Force vs. Voltage ' strrep(varName, '_', '.')];
    hold on
end
hold off
xlabel('Measured Voltage [V]')
ylabel('Applied Force to Sensor [N]')
legend(legendTitles, 'Location', 'northwest');  % Add a legend with descriptive labels
title('Log-Log Scale Comparison of Voltage and Force');      % Add a title

% Take the exponent of the voltage 
vOutExp = exp(vOut);
expVoltageOut = exp(voltageOut)

% With basic curve fitting we can get the coefficients of the nonlinear
% Relationship with a polynomial with R=1 as:
force_est = 1.60717263158763e-09*vOutExp.^5 ...
    - 3.97326895848675e-07 * vOutExp.^4 ...
    + 3.48267292566989e-05 * vOutExp.^3 ...
    - 0.00117407617298212 * vOutExp.^2 ...
    + 0.0234485875465875 * vOutExp	...
    - 0.0225118029465446;

forceEstimate = 1.60717263158763e-09*expVoltageOut.^5 ...
    - 3.97326895848675e-07 * expVoltageOut.^4 ...
    + 3.48267292566989e-05 * expVoltageOut.^3 ...
    - 0.00117407617298212 * expVoltageOut.^2 ...
    + 0.0234485875465875 * expVoltageOut	...
    - 0.0225118029465446

% Plot the actual TData vs the estimated forces using the measured voltage
figure
legendTitles = {};
for i = 1:length(variableNames)
    varName = variableNames{i};
    legendTitles{end+1} = ['Force vs. Voltage ' strrep(varName, '_', '.')];
    
    plot((voltageOut.(varName)),(force),'linewidth',3)
    hold on
    plot(voltageOut.(varName),force_est,':','linewidth',3)
    hold on    
    %loglog(TData.(varName), force, 'LineWidth', (0.5*i)+1, 'Color', colors(i, :));
end
hold off
xlabel('Measured Voltage [V]')
ylabel('Force [N]')
title('Comparison of TData and Estimated Forces');
legend('TData','Estimated')
axis([0 5 0 5])