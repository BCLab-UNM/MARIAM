% FSR_force sensor calibration
clear all
close all

% Check if the folder 'figures' exists, create it if it doesn't
folderName = 'figures';
if ~exist(folderName, 'dir')
    mkdir(folderName);  % Make a new directory if it doesn't exist
end

% Read data from table
% Specify the path to the CSV file
filename = 'fsr_calibration_averages-2200Ohm.csv';
TData = readtable(filename);

% Get data meta-data
resistorNames = TData.Properties.VariableNames(2:end);
totalResistors = size(resistorNames, 2);
totalForces = size(resistorNames, 1);
colors = summer(totalResistors);

% Assign voltageOut and Force data from file
voltageOut = TData(:, 2:end);
force = (TData.Grams / 1000) * 9.81;

% Plot observed data
legendTitles = {};
figure;
for i = 1:length(resistorNames)
    varName = resistorNames{i};
    plot(voltageOut.(varName), force, 'LineWidth', 2, 'Color', colors(i, :));
    legendTitles{end+1} = [strrep(varName(2:end), '_', '.') ' Ohm'];
    hold on;
end
hold off;
xlabel('Measured Voltage [V]');
ylabel('Applied Force to Sensor [N]');
legend(legendTitles, 'Location', 'northwest');
title('Observed Voltage vs. Force');
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'ObservedVoltageVsForcePlot.png'));

% Log-log scale plot
legendTitles = {};
figure;
for i = 1:length(resistorNames)
    varName = resistorNames{i};
    loglog(voltageOut.(varName), force, 'LineWidth', 2, 'Color', colors(i, :));
    legendTitles{end+1} = [strrep(varName(2:end), '_', '.') ' Ohm'];
    hold on;
end
hold off;
xlabel('Measured Voltage [V]');
ylabel('Applied Force to Sensor [N]');
legend(legendTitles, 'Location', 'northwest');
title('Log-Log Scale Comparison: Voltage vs. Force');
axis([0 5 0 5]);
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'LogLogVoltageVsForcePlot.png'));

% Exponential transformation of voltage
expVoltageOut = voltageOut;

% Plot force against exponent of voltages for curve fitting
legendTitles = {};
figure;
for i = 1:length(resistorNames)
    varName = resistorNames{i};
    plot(expVoltageOut.(varName), force, 'LineWidth', 2);
    legendTitles{end+1} = [strrep(varName(2:end), '_', '.') ' Ohm'];
    hold on;
end
hold off;
xlabel('Exponential of Measured Voltage [V]');
ylabel('Applied Force to Sensor [N]');
legend(legendTitles, 'Location', 'northwest');
title('Exponential Voltage vs. Force');
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'ExponentialVoltageVsForcePlot.png'));

% Fit a 3rd-degree polynomial and store coefficients
coeffMatrix = zeros(length(resistorNames), 4);
for i = 1:length(resistorNames)
    dataColumn = expVoltageOut{:, i};
    coeff = polyfit(dataColumn, force, 3);
    coeffMatrix(i, :) = coeff;
end
coeffTable = array2table(coeffMatrix', 'VariableNames', resistorNames);

% Estimate force using polynomial coefficients at the measured points (existing computation)
for i = 1:totalResistors
    forceEstimateArray(:, i) = ...
       coeffTable{1, i} * expVoltageOut{:, i}.^3 ...
     + coeffTable{2, i} * expVoltageOut{:, i}.^2 ...
     + coeffTable{3, i} * expVoltageOut{:, i} ...
     + coeffTable{4, i};
end
forceEstimate = array2table(forceEstimateArray, 'VariableNames', expVoltageOut.Properties.VariableNames);

% Plot observed vs estimated forces using a continuous fitted curve from 0 to 5 V
figure;
legendTitles = {};
x_fit = linspace(0, 5, 100);  % Continuous voltage values from 0 to 5 V
for i = 1:length(resistorNames)
    varName = resistorNames{i};
    % Plot observed data points
    plot(voltageOut.(varName), force, 'linewidth', 1, 'Color', colors(i, :));
    legendTitles{end+1} = ['Observed ' strrep(varName(2:end), '_', '.') ' Ohm'];
    hold on;
    % Compute the fitted curve using the polynomial coefficients
    p = coeffMatrix(i, :);  % p(1) = coeff for x^3, p(2) for x^2, p(3) for x, p(4) constant
    force_fit = p(1)*x_fit.^3 + p(2)*x_fit.^2 + p(3)*x_fit + p(4);
    % Plot the continuous fitted curve
    plot(x_fit, force_fit, ':', 'linewidth', 3, 'Color', colors(i, :));
    legendTitles{end+1} = ['Estimated ' strrep(varName(2:end), '_', '.') ' Ohm'];
end
hold off;
xlabel('Measured Voltage [V]');
ylabel('Force [N]');
title('Observed and Estimated Forces Compared');
legend(legendTitles, 'Location', 'northwest');
axis([0 5 0 5]);
set(gcf, 'Position', [100, 100, 800, 600]);
saveas(gcf, fullfile('figures', 'ObservedvsEstimatedComparedVoltageVsForcePlot.png'));

% Display the coefficients
disp(coeffTable);
