% Read the CSV file
input_filename = 'fsrCalibrationTests-2200Ohm.csv';
data = readtable(input_filename);

% Compute the average across all sensor columns (excluding the first column)
averages = mean(data{:, 2:end}, 2);

% Create a new table with the averaged results
output_table = table(data.Grams, averages, 'VariableNames', {'Grams', '2.2k'});

% Write the new table to a CSV file
output_filename = 'fsrCalibrationAverage-2200Ohm.csv';
writetable(output_table, output_filename);

fprintf('Averaged data saved to %s\n', output_filename);
