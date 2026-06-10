clc; clear; close all;

% ===============================
% PARAMETERS
% ===============================
freqs = [0.2, 0.22, 0.25,0.27,0.3,0.32,0.35,0.37,0.4];
phase = 0;   % change to 0 or 1
spineAmpList=[0];
spineStiffList={'rigid'};
%spineAmpList = [0,1,2,3,4,5,6,7,8,9,10];
%spineStiffList = {5.184,6.048,6.912,7.776, 8.64,9.504,10.36,11.232,12.096};
basePath = "C:\Users\jo_dr\OneDrive\Documents\CogSci\8SoSe26\RobIguana\Simulation\SimExecutable\SMARCUnityHDRP2_Data";

saveFigPath = fullfile(basePath, 'TrajectoryPlots');
if ~exist(saveFigPath, 'dir')
    mkdir(saveFigPath);
end

csvFile = fullfile(basePath, 'experiment_summary.csv');

% ===============================
% CREATE CSV FILE IF NOT EXISTS
% ===============================
if ~exist(csvFile, 'file')
    headerTbl = cell2table(cell(0,8), ...
        'VariableNames', {'phase','spinemode','frequency','spineAmp','spineStiff','speed','CoT','deviation'});
    writetable(headerTbl, csvFile);
end

% ===============================
% MAIN LOOP
% ===============================
for sIdx = 1:length(spineStiffList)
    currentStiff = spineStiffList{sIdx};

    for aIdx = 1:length(spineAmpList)
        currentAmp = spineAmpList(aIdx);

        % -------------------------------
        % Determine spine mode and stored values
        % -------------------------------
        if ischar(currentStiff) || isstring(currentStiff)
            if strcmpi(currentStiff, 'rigid')
                spinemode = 'rigid';
                ampRecord = 0;
                stiffRecord = 0;
                ampForFile = 0;
                stiffForFile = 'rigid';
            else
                warning('Unknown string spine stiffness mode: %s. Skipping.', string(currentStiff));
                continue;
            end
        else
            if currentAmp < 1
                spinemode = 'flexible';
            else
                spinemode = 'active';
            end
            ampRecord = currentAmp;
            stiffRecord = currentStiff;
            ampForFile = currentAmp;
            stiffForFile = currentStiff;
        end

        % -------------------------------
        % Create figure for this combo
        % -------------------------------
        fig = figure('Visible','off');
        tiledlayout(length(freqs), 1, 'TileSpacing', 'compact', 'Padding', 'compact');

        resultsCell = {};

        for i = 1:length(freqs)

            f = freqs(i);
            p = phase;

            % -------------------------------
            % Build filenames
            % -------------------------------
            if ischar(stiffForFile) || isstring(stiffForFile)
                positionFilename = fullfile(basePath, ...
                    sprintf('position_f%.2f_phase%.1f_amp%.1f_stiff_%s.csv', f, p, ampForFile, char(stiffForFile)));
                energyFilename = fullfile(basePath, ...
                    sprintf('energy_f%.2f_phase%.1f_amp%.1f_stiff_%s.csv', f, p, ampForFile, char(stiffForFile)));
            else
                positionFilename = fullfile(basePath, ...
                    sprintf('position_f%.2f_phase%.1f_amp%.1f_stiff_%.2f.csv', f, p, ampForFile, stiffForFile));
                energyFilename = fullfile(basePath, ...
                    sprintf('energy_f%.2f_phase%.1f_amp%.1f_stiff_%.2f.csv', f, p, ampForFile, stiffForFile));
            end

            % -------------------------------
            % Check if files exist
            % -------------------------------
            if ~exist(positionFilename, 'file')
                warning('Missing position file: %s', positionFilename);
                nexttile;
                title(sprintf('Missing position file for f = %.2f Hz', f));
                axis off;
                continue;
            end

            if ~exist(energyFilename, 'file')
                warning('Missing energy file: %s', energyFilename);
                nexttile;
                title(sprintf('Missing energy file for f = %.2f Hz', f));
                axis off;
                continue;
            end

            % -------------------------------
            % Load data
            % -------------------------------
            data = readtable(positionFilename);
            energyData = readtable(energyFilename);

            t = data{:,1};

            % safer reference row
            refIdx = min(500, size(data,1));

            x = 10 * (data{:,2} - data{refIdx,2});
            z = 10 * (data{:,4} - data{refIdx,4});

            % ===============================
            % TIME WINDOWS
            % ===============================
            t1 = 5; %#ok<NASGU>
            t2 = 5 + 20*(1/f);
            t3 = 5 + 25*(1/f);

            idx_fit  = (t >= t2) & (t <= t3);
            idx_eval = (t > t3);

            if nnz(idx_fit) < 2 || nnz(idx_eval) < 2
                warning('Not enough data points for fitting/evaluation: %s', positionFilename);
                nexttile;
                plot(x, z, 'LineWidth', 1.2);
                grid on;
                title(sprintf('Insufficient data, f = %.2f Hz', f), 'FontSize', 8);
                ylabel(sprintf('f = %.2f Hz', f));
                continue;
            end

            % ===============================
            % ENERGY CALCULATION
            % ===============================
            t_energy = energyData{:,1};
            E_energy = energyData{:,2};

            [~, idx_t3_energy] = min(abs(t_energy - t3));
            totalEnergy = E_energy(end) - E_energy(idx_t3_energy);

            % ===============================
            % LINEAR FIT
            % ===============================
            x_fit = x(idx_fit);
            z_fit = z(idx_fit);

            p_fit = polyfit(x_fit, z_fit, 1);
            a = p_fit(1);
            b = p_fit(2);

            % ===============================
            % DEVIATION
            % New definition:
            % deviation = old RMS perpendicular deviation /
            %             projected length of eval segment on reference line
            % ===============================
            x_eval = x(idx_eval);
            z_eval = z(idx_eval);

            % old RMS perpendicular deviation
            dist = abs(a*x_eval - z_eval + b) ./ sqrt(a^2 + 1);
            oldDeviation = sqrt(mean(dist.^2));

            % projected length along fitted reference line
            u = [1, a] / sqrt(1 + a^2);   % unit direction vector of line
            s_eval = x_eval * u(1) + z_eval * u(2);
            projectedLength = abs(s_eval(end) - s_eval(1));

            if projectedLength < eps
                deviation = NaN;
            else
                deviation = oldDeviation / projectedLength;
                deviation = atan(deviation)*180/pi;
            end

            % ===============================
            % SPEED AND COT USING TRAJECTORY LENGTH
            % ===============================
            t_eval = t(idx_eval);

            dx = diff(x_eval);
            dz = diff(z_eval);
            trajLength = sum(sqrt(dx.^2 + dz.^2));

            dt_eval = t_eval(end) - t_eval(1);

            if abs(dt_eval) < eps
                speed = NaN;
            else
                speed = trajLength / dt_eval;
            end

            if abs(trajLength) < eps
                COT = NaN;
            else
                COT = totalEnergy / trajLength;
            end

            % ===============================
            % STORE RESULT
            % ===============================
            resultsCell(end+1,:) = {p, spinemode, f, ampRecord, stiffRecord, speed, COT, deviation}; %#ok<SAGROW>

            % ===============================
            % PLOTTING
            % ===============================
            nexttile;
            plot(x, z, 'LineWidth', 1.2); hold on;

            x_line = linspace(min(x), max(x), 100);
            z_line = a*x_line + b;
            plot(x_line, z_line, 'r--', 'LineWidth', 1.5);

            grid on;
            xlim([0, 14]);
            ylim([min(z), max(z)]);

            ylabel(sprintf('f = %.2f Hz', f));

            if i == length(freqs)
                xlabel('x displacement');
            end

            title(sprintf('Phase = %.1f, Norm Dev = %.5f, Speed = %.3f m/s, COT = %.3f J/m', ...
                p, deviation, speed, COT), 'FontSize', 8);
        end

        % -------------------------------
        % Overall title
        % -------------------------------
        sgtitle(sprintf(' Phase = %.1f | Mode = %s | Amp = %.1f | Stiff = %s', ...
            phase, spinemode, ampRecord, num2str(stiffRecord)));

        % -------------------------------
        % Save figure
        % -------------------------------
        figName = sprintf('traj_phase%.1f_mode_%s_amp%.1f_stiff_%s.png', ...
            phase, spinemode, ampRecord, num2str(stiffRecord));
        saveas(fig, fullfile(saveFigPath, figName));

        close(fig);

        % -------------------------------
        % Append results to CSV
        % -------------------------------
        if ~isempty(resultsCell)
            resultsTbl = cell2table(resultsCell, ...
                'VariableNames', {'phase','spinemode','frequency','spineAmp','spineStiff','speed','CoT','deviation'});
            writetable(resultsTbl, csvFile, 'WriteMode', 'append');
        end
    end
end

disp('All figures saved and experiment data appended to CSV.');