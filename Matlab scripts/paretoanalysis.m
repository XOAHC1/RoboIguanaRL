clc; clear; close all;

%% =========================
% 0. Plot colour and style settings
%% =========================
% Set all default text to Arial
set(groot, 'defaultAxesFontName', 'Arial');
set(groot, 'defaultTextFontName', 'Arial');
set(groot, 'defaultLegendFontName', 'Arial');
set(groot, 'defaultColorbarFontName', 'Arial');

% Main distance colormap:
% low distance = blue, medium distance = gold, high distance = rose
distancePalette = "coord_blue_gold_rose";

% Colour scaling
% This restores the previous colour appearance.
% Distances below 5 use the lowest colour.
% Distances above 20 use the highest colour.
manualColorLimits = [5 20];

% Marker colours
bestFaceColor   = [0.00 0.58 0.46];   % distinct emerald-green accent
idealColor      = [0.46 0.28 0.64];   % purple accent, distinct from data colormap
edgeColor       = [0.06 0.06 0.06];   % almost black
helperLineColor = [0.55 0.55 0.55];

% Font sizes
axisFontSize   = 9.5;
labelFontSize  = 11;
legendFontSize = 9;
cbFontSize     = 9.5;

% Display controls
showHelperLines = false;

%% =========================
% 1. Read experiment_summary.csv
%% =========================
filename = 'experiment_summary_full.csv';
T = readtable(filename, 'TextType', 'string');

% Keep only rows with valid numeric outputs
validRows = isfinite(T.speed) & isfinite(T.CoT) & isfinite(T.deviation);
T = T(validRows, :);

%% =========================
% 2. For each output, take 5th to 95th percentile,
%    then calculate mean and standard deviation
%% =========================
speed = T.speed;
CoT = T.CoT;
deviation = T.deviation;

% Speed
P5_speed  = prctile(speed, 5);
P95_speed = prctile(speed, 95);
keepRows_speed = (speed >= P5_speed) & (speed <= P95_speed);
mu_speed = mean(speed(keepRows_speed), 'omitnan');
sigma_speed = std(speed(keepRows_speed), 'omitnan');

% CoT
P5_CoT  = prctile(CoT, 5);
P95_CoT = prctile(CoT, 95);
keepRows_CoT = (CoT >= P5_CoT) & (CoT <= P95_CoT);
mu_CoT = mean(CoT(keepRows_CoT), 'omitnan');
sigma_CoT = std(CoT(keepRows_CoT), 'omitnan');

% Deviation
P5_deviation  = prctile(deviation, 5);
P95_deviation = prctile(deviation, 95);
keepRows_deviation = (deviation >= P5_deviation) & (deviation <= P95_deviation);
mu_deviation = mean(deviation(keepRows_deviation), 'omitnan');
sigma_deviation = std(deviation(keepRows_deviation), 'omitnan');

% Avoid division by zero
if sigma_speed == 0
    sigma_speed = 1;
end
if sigma_CoT == 0
    sigma_CoT = 1;
end
if sigma_deviation == 0
    sigma_deviation = 1;
end

%% =========================
% 3. Normalize three outputs
%% =========================
speed_norm     = (speed - mu_speed) ./ sigma_speed;
CoT_norm       = (CoT - mu_CoT) ./ sigma_CoT;
deviation_norm = (deviation - mu_deviation) ./ sigma_deviation;

% Convert to "larger is better" scores
% speed: maximize
% CoT: minimize
% deviation: minimize
speed_score     = speed_norm;
CoT_score       = -CoT_norm;
deviation_score = -deviation_norm;

scores = [speed_score, CoT_score, deviation_score];

%% =========================
% 4. Find ideal point and shortest-distance cases
%% =========================
% Ideal point in normalized score space
idealPoint = [max(speed_score), max(CoT_score), max(deviation_score)];

% Distance of every point to ideal point
distToIdeal = sqrt(sum((scores - idealPoint).^2, 2));

% Ranked cases by shortest distance
[~, sortedIdx] = sort(distToIdeal, 'ascend');

% Best compromise point
bestIdx = sortedIdx(1);

% Best 3 cases for console output only
nBest = min(10, height(T));
best3Idx = sortedIdx(1:nBest);

fprintf('Best compromise point index: %d\n', bestIdx);
fprintf('Best compromise point values:\n');
fprintf('  speed     = %.6f\n', speed(bestIdx));
fprintf('  CoT       = %.6f\n', CoT(bestIdx));
fprintf('  deviation = %.6f\n', deviation(bestIdx));

if any(strcmpi(T.Properties.VariableNames, 'frequency'))
    fprintf('  frequency = %.6f\n', T.frequency(bestIdx));
end
if any(strcmpi(T.Properties.VariableNames, 'spineAmp'))
    fprintf('  spineAmp  = %.6f\n', T.spineAmp(bestIdx));
end
if any(strcmpi(T.Properties.VariableNames, 'spineStiff'))
    if isnumeric(T.spineStiff)
        fprintf('  spineStiff = %.6f\n', T.spineStiff(bestIdx));
    else
        disp("  spineStiff = " + string(T.spineStiff(bestIdx)));
    end
end
if any(strcmpi(T.Properties.VariableNames, 'spinemode'))
    disp("  spinemode  = " + string(T.spinemode(bestIdx)));
end

%% =========================
% 5. Colour scaling adjustment
%% =========================
% Previous colour style:
% colour mapping is linear between cLow and cHigh.
% Values below cLow are clipped to the lowest colour.
% Values above cHigh are clipped to the highest colour.
cLow  = manualColorLimits(1);
cHigh = manualColorLimits(2);

if cHigh <= cLow
    cLow  = min(distToIdeal);
    cHigh = max(distToIdeal);
end

% Clip colour values only for visual mapping.
% The actual distToIdeal values are unchanged.
distColor = distToIdeal;
distColor(distColor < cLow)  = cLow;
distColor(distColor > cHigh) = cHigh;

%% =========================
% 6. Draw trade-off plot
%% =========================
% Raw-metric ideal point for visual reference only
rawIdealPoint = [max(speed), min(CoT), min(deviation)];

% Sort for cleaner plotting so farther points are drawn first
[~, plotOrder] = sort(distToIdeal, 'descend');

% Figure
fig = figure('Color', 'w', ...
    'Units', 'centimeters', ...
    'Position', [2 2 16 12], ...
    'Renderer', 'painters');

fig.InvertHardcopy = 'off';

ax = axes(fig, ...
    'Units', 'normalized', ...
    'Position', [0.105 0.135 0.665 0.785]);

hold(ax, 'on');

% Apply coordinated thesis colormap
colormap(ax, makeThesisColormap(distancePalette, 256));

% All cases
hAll = scatter3(ax, ...
    speed(plotOrder), CoT(plotOrder), deviation(plotOrder), ...
    18, distColor(plotOrder), 'filled', ...
    'MarkerEdgeColor', 'none');

try
    hAll.MarkerFaceAlpha = 0.80;
catch
end

%% =========================
% 7. Best case marker: solid star
%% =========================
hBest = scatter3(ax, ...
    speed(bestIdx), CoT(bestIdx), deviation(bestIdx), ...
    180, 'p', ...
    'filled', ...
    'MarkerFaceColor', bestFaceColor, ...
    'MarkerEdgeColor', edgeColor, ...
    'LineWidth', 1.0);

%% =========================
% 8. Ideal reference marker: x
%% =========================
hIdeal = scatter3(ax, ...
    rawIdealPoint(1), rawIdealPoint(2), rawIdealPoint(3), ...
    120, 'x', ...
    'MarkerEdgeColor', idealColor, ...
    'LineWidth', 2.0);

%% =========================
% 9. Axis labels and formatting
%% =========================
xlabel(ax, 'Speed (m/s)', ...
    'FontName', 'Arial', ...
    'FontSize', labelFontSize);

ylabel(ax, 'CoT (J/m)', ...
    'FontName', 'Arial', ...
    'FontSize', labelFontSize);

zlabel(ax, 'Deviation (deg)', ...
    'FontName', 'Arial', ...
    'FontSize', labelFontSize);

set(ax, ...
    'FontName', 'Arial', ...
    'FontSize', axisFontSize, ...
    'LineWidth', 0.85, ...
    'Box', 'on', ...
    'Layer', 'top', ...
    'TickDir', 'in', ...
    'TickLength', [0.012 0.012]);

% Clean, subtle grid
grid(ax, 'on');
ax.GridAlpha = 0.10;
ax.MinorGridAlpha = 0.04;
ax.XMinorGrid = 'off';
ax.YMinorGrid = 'off';
ax.ZMinorGrid = 'off';

view(ax, 38, 22);
pbaspect(ax, [1.15 1 0.85]);

% Tight but readable limits
xMargin = 0.05 * range(speed);
yMargin = 0.05 * range(CoT);
zMargin = 0.05 * range(deviation);

if xMargin == 0, xMargin = 1; end
if yMargin == 0, yMargin = 1; end
if zMargin == 0, zMargin = 1; end

xlim(ax, [min(speed)-xMargin, max(speed)+xMargin]);
ylim(ax, [min(CoT)-yMargin, max(CoT)+yMargin]);
zlim(ax, [min(deviation)-zMargin, max(deviation)+zMargin]);

%% =========================
% 10. Optional helper lines
%% =========================
if showHelperLines
    xmin = ax.XLim(1);
    ymin = ax.YLim(1);
    zmin = ax.ZLim(1);

    plot3(ax, [speed(bestIdx) speed(bestIdx)], ...
              [CoT(bestIdx) CoT(bestIdx)], ...
              [zmin deviation(bestIdx)], ...
              '--', 'Color', helperLineColor, 'LineWidth', 0.75);

    plot3(ax, [speed(bestIdx) speed(bestIdx)], ...
              [ymin CoT(bestIdx)], ...
              [deviation(bestIdx) deviation(bestIdx)], ...
              '--', 'Color', helperLineColor, 'LineWidth', 0.75);

    plot3(ax, [xmin speed(bestIdx)], ...
              [CoT(bestIdx) CoT(bestIdx)], ...
              [deviation(bestIdx) deviation(bestIdx)], ...
              '--', 'Color', helperLineColor, 'LineWidth', 0.75);
end

%% =========================
% 11. Colourbar
%% =========================
% Use the same clipped colour scale as the previous preferred version
caxis(ax, [cLow cHigh]);

cb = colorbar(ax);
cb.FontName = 'Arial';
cb.FontSize = cbFontSize;
cb.LineWidth = 0.75;
cb.TickDirection = 'in';
cb.TickLabelInterpreter = 'tex';

cb.Label.String = 'Normalized distance to ideal reference point';
cb.Label.FontName = 'Arial';
cb.Label.FontSize = cbFontSize;

% Manual colourbar position
cb.Units = 'normalized';
cb.Position = [0.815 0.245 0.026 0.560];

% Clear tick labels showing clipping
cb.Ticks = linspace(cLow, cHigh, 4);
cb.TickLabels = { ...
    sprintf('\\leq %.0f', cLow), ...
    sprintf('%.0f', cb.Ticks(2)), ...
    sprintf('%.0f', cb.Ticks(3)), ...
    sprintf('\\geq %.0f', cHigh)};

%% =========================
% 12. Legend
%% =========================
lgd = legend(ax, [hAll, hBest, hIdeal], ...
    {'All cases', 'Best case', 'Ideal reference'}, ...
    'Box', 'off', ...
    'FontName', 'Arial', ...
    'FontSize', legendFontSize);

lgd.ItemTokenSize = [16 8];
lgd.Units = 'normalized';

% Move legend slightly left and upward
lgd.Position = [0.08 0.85 0.17 0.09];

%% =========================
% 13. Save figure
%% =========================
exportgraphics(fig, 'tradeoff_restored_colors_3D.png', 'Resolution', 600);
exportgraphics(fig, 'tradeoff_restored_colors_3D.pdf', 'ContentType', 'vector');

%% =========================
% 14. Build best-3 results table
%% =========================
Best3Table = T(best3Idx, :);
Best3Table.distToIdeal = distToIdeal(best3Idx);
Best3Table.rank = (1:nBest)';

% Move rank and distance to front
Best3Table = movevars(Best3Table, {'rank', 'distToIdeal'}, 'Before', 1);

disp('Best 3 shortest-distance cases:');
disp(Best3Table);

%% =========================
% 15. For each spinemode and frequency, find best spineAmp and spineStiff
%% =========================
requiredVars = {'spinemode','frequency','spineAmp','spineStiff'};
missingVars = requiredVars(~ismember(requiredVars, T.Properties.VariableNames));

if ~isempty(missingVars)
    error('Missing required column(s): %s', strjoin(missingVars, ', '));
end

% Group by spinemode and frequency
[G, groupSpinemode, groupFrequency] = findgroups(string(T.spinemode), T.frequency);
nGroups = max(G);

bestRowIdx_group     = zeros(nGroups,1);
bestSpineAmp_group   = strings(nGroups,1);
bestSpineStiff_group = strings(nGroups,1);
bestDist_group       = zeros(nGroups,1);
bestSpeed_group      = zeros(nGroups,1);
bestCoT_group        = zeros(nGroups,1);
bestDeviation_group  = zeros(nGroups,1);

for g = 1:nGroups
    idxGroup = find(G == g);

    % Find the row with shortest distance within this spinemode-frequency group
    [bestDist_group(g), loc] = min(distToIdeal(idxGroup));
    idxBestGroup = idxGroup(loc);

    bestRowIdx_group(g)     = idxBestGroup;
    bestSpineAmp_group(g)   = string(T.spineAmp(idxBestGroup));
    bestSpineStiff_group(g) = string(T.spineStiff(idxBestGroup));
    bestSpeed_group(g)      = T.speed(idxBestGroup);
    bestCoT_group(g)        = T.CoT(idxBestGroup);
    bestDeviation_group(g)  = T.deviation(idxBestGroup);
end

% Build summary table
BestPerModeFreq = table( ...
    groupSpinemode, ...
    groupFrequency, ...
    bestRowIdx_group, ...
    bestSpineAmp_group, ...
    bestSpineStiff_group, ...
    bestDist_group, ...
    bestSpeed_group, ...
    bestCoT_group, ...
    bestDeviation_group, ...
    'VariableNames', { ...
    'spinemode', ...
    'frequency', ...
    'bestRowIndex', ...
    'bestSpineAmp', ...
    'bestSpineStiff', ...
    'distToIdeal', ...
    'speed', ...
    'CoT', ...
    'deviation'} );

% Sort nicely
BestPerModeFreq = sortrows(BestPerModeFreq, {'spinemode','frequency'});

disp('Best spineAmp and spineStiff for each spinemode-frequency combination:');
disp(BestPerModeFreq);

%% ========================================================================
% Local functions
%% ========================================================================

function cmap = makeThesisColormap(paletteName, n)
% makeThesisColormap creates a smooth colormap from coordinated anchor colours.

    if nargin < 2
        n = 256;
    end

    anchors = getPaletteAnchors(paletteName);

    xAnchors = linspace(0, 1, size(anchors, 1));
    xQuery   = linspace(0, 1, n);

    cmap = interp1(xAnchors, anchors, xQuery, 'pchip');
    cmap = max(min(cmap, 1), 0);
end

function c = getThesisColor(paletteName, anchorIndex)
% getThesisColor returns one anchor colour from a selected coordinated palette.

    anchors = getPaletteAnchors(paletteName);
    anchorIndex = max(1, min(anchorIndex, size(anchors, 1)));
    c = anchors(anchorIndex, :);
end

function anchors = getPaletteAnchors(paletteName)
% getPaletteAnchors stores coordinated thesis colour palettes.

    switch string(paletteName)

        case "coord_blue"
            anchors = [
                0.968 0.978 0.982
                0.840 0.905 0.920
                0.675 0.805 0.825
                0.500 0.690 0.720
                0.340 0.560 0.610
                0.210 0.430 0.500
                0.110 0.300 0.390
            ];

        case "coord_gold"
            anchors = [
                0.985 0.982 0.965
                0.930 0.905 0.790
                0.855 0.790 0.620
                0.760 0.670 0.470
                0.635 0.545 0.335
                0.500 0.425 0.245
                0.355 0.305 0.180
            ];

        case "coord_rose"
            anchors = [
                0.955 0.900 0.890
                0.920 0.800 0.780
                0.870 0.685 0.655
                0.790 0.560 0.530
                0.680 0.430 0.410
                0.550 0.305 0.295
                0.410 0.190 0.190
            ];

        case "coord_blue_gold_rose"
            anchors = [
                0.110 0.300 0.390
                0.210 0.430 0.500
                0.340 0.560 0.610
                0.500 0.690 0.720
                0.760 0.670 0.470
                0.855 0.790 0.620
                0.790 0.560 0.530
                0.680 0.430 0.410
                0.550 0.305 0.295
                0.410 0.190 0.190
            ];

        otherwise
            error('Unknown palette name: %s. Use "coord_blue", "coord_gold", "coord_rose", or "coord_blue_gold_rose".', paletteName);
    end
end