%FSR_force sensor calibration
clear all
close all

% Check if the folder 'figures' exists, create it if it doesn't
folderName = 'figures';
if ~exist(folderName, 'dir')
    mkdir(folderName);  % Make a new directory if it doesn't exist
end

% Read data from table
% Specify the path to the CSV file
filename = 'fsrCalibrationAverages1.csv';
TData = readtable(filename);

% Get data meta-data
resistorNames = TData.Properties.VariableNames(2:end);
totalResistors = size(resistorNames, 2);
totalForces = size(resistorNames, 1);
colors = cool(totalResistors);

% Assign voltageOut and Force data from file
voltageOut = TData(:, 2:end)
force = (TData.Resistors / 1000)*9.81

% Plot observed data
legendTitles = {};
figure
for i = 1:length(resistorNames)
    varName = resistorNames{i};
    plot(voltageOut.(varName), force, 'LineWidth', 2, 'Color', colors(i, :));
    legendTitles{end+1} = [strrep(varName(2:end), '_', '.') ' Ohm' ];
    hold on
end
hold off
xlabel('Measured Voltage [V]')
ylabel('Applied Force to Sensor [N]')
legend(legendTitles, 'Location', 'northwest');
title('Observed Voltage vs. Force');
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'ObservedVoltageVsForcePlot.png'));

% Log-log scale plot we can see this relationship between voltage and force is nonlinear
legendTitles = {};
figure
for i = 1:length(resistorNames)
    varName = resistorNames{i};
    loglog(voltageOut.(varName), force, 'LineWidth', 2, 'Color', colors(i, :));
    legendTitles{end+1} = [strrep(varName(2:end), '_', '.') ' Ohm' ];
    hold on
end
hold off
xlabel('Measured Voltage [V]')
ylabel('Applied Force to Sensor [N]')
legend(legendTitles, 'Location', 'northwest');
title('Log-Log Scale Comparison: Voltage vs. Force');
axis([0 5 0 5])
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'LogLogVoltageVsForcePlot.png'));

% Take the exponent of the voltage 
expVoltageOut = exp(voltageOut)

% Plot force against exponent of voltages for curve fitting
legendTitles = {};
figure
for i = 1:length(resistorNames)
    varName = resistorNames{i};
    plot(expVoltageOut.(varName), force, 'LineWidth', 2, 'Color', colors(i, :));
    legendTitles{end+1} = [strrep(varName(2:end), '_', '.') ' Ohm' ];
    hold on
end
hold off
xlabel('Exponential of Measured Voltage [V]')
ylabel('Applied Force to Sensor [N]')
legend(legendTitles, 'Location', 'northwest'); % Add a legend with descriptive labels
title('Exponential Voltage vs. Force'); % Add a title
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'ExponentialVoltageVsForcePlot.png'));

% Determine coef for each resistor
% Fit a 5th-degree polynomial and store coefficients
coeffMatrix = zeros(length(resistorNames), 6);  % 6 coefficients for a 5th-degree polynomial
for i = 1:length(resistorNames)
    dataColumn = expVoltageOut{:, i};
    coeff = polyfit(dataColumn, force, 5);
    coeffMatrix(i, :) = coeff;
end
coeffTable = array2table(coeffMatrix', 'VariableNames', resistorNames)

% With basic curve fitting we can get the coefficients of the nonlinear
% Relationship with a polynomial with R=1 as:
for i = 1:totalResistors
    forceEstimateArray(:, i) = ...
       coeffTable{1, i} * expVoltageOut{:, i}.^5 ...
     + coeffTable{2, i} * expVoltageOut{:, i}.^4 ...
     + coeffTable{3, i} * expVoltageOut{:, i}.^3 ...
     + coeffTable{4, i} * expVoltageOut{:, i}.^2 ...
     + coeffTable{5, i} * expVoltageOut{:, i}	...
     + coeffTable{6, i};
end
forceEstimate = array2table(forceEstimateArray, 'VariableNames', expVoltageOut.Properties.VariableNames)

% Plot the observed vs the estimated forces using the measured voltage
figure
legendTitles = {};
for i = 1 : length(resistorNames)
    varName = resistorNames{i};
    
    plot((voltageOut.(varName)),(force),'linewidth',1,'Color', colors(i, :));
    legendTitles{end+1} = ['Observed ' strrep(varName(2:end), '_', '.') ' Ohm' ];
    hold on
    plot(voltageOut.(varName),forceEstimate{:, i},':','linewidth',3,'Color', colors(i, :));
    legendTitles{end+1} = ['Estimated ' strrep(varName(2:end), '_', '.') ' Ohm' ];
    hold on    
end
hold off
xlabel('Measured Voltage [V]')
ylabel('Force [N]')
title('Observed and Estimated Forces Compared');
legend(legendTitles, 'Location', 'northwest');
axis([0 5 0 5])
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'ObservedvsEstimatedComparedVoltageVsForcePlot.png'));