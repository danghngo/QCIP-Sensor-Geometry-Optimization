clc; clear; close all;

% PHASE C.1 - ONE-VIEW DIMENSION-OPTIMIZED SWEEP
% Sweep:
%   azimuth   = 130:2:170
%   elevation = [5 10 15 20 25 30]
%
% Fixed:
%   baseline  = 0.15
%   HFOV      = 24 deg
%   range     = 18 m
%
% Total trials:
%   21 x 6 = 126

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir), thisDir = pwd; end
addpath(genpath(thisDir)); rehash;

timestampTag = char(string(datetime('now','Format','yyyyMMdd_HHmmss')));
outRoot = fullfile(thisDir, 'outputs', ['phaseC1_one_view_dimensional_' timestampTag]);
resultsDir = fullfile(outRoot, 'results');
plotsDir = fullfile(outRoot, 'plots');
pcDir = fullfile(outRoot, 'pointclouds');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
if ~exist(plotsDir, 'dir'), mkdir(plotsDir); end
if ~exist(pcDir, 'dir'), mkdir(pcDir); end

azimuthList   = 130:2:170;
elevationList = [5 10 15 20 25 30];
baselineVal   = 0.15;
hfovDegVal    = 24;
rangeVal      = 18;
imageSizeVal  = [720 1280];

disparityRangeVal      = [16 64];
maxDepthVal            = 30;
minValidDisparityVal   = 16;
backgroundThresholdVal = 245;
useAdaptiveHistEqVal   = true;

cropXVal = [-0.2 8.8];
cropYVal = [-1.4 1.4];
cropZVal = [0.0 2.8];

gtDims.length = 8.4;
gtDims.width  = 1.2;
gtDims.height = 1.52;

bootstrapSamples   = 1000;
maxBootstrapPoints = 20000;
clearanceQuantile  = 99;
safetyMargin       = 0.0;

gtMeshSpacingVal      = 0.10;
maxEvalPointsVal      = 30000;
chunkSizeNNVal        = 300;
chunkSizeMeshVal      = 120;
triangleCandidatesVal = 12;

rows = [];
heightErrorGrid = nan(numel(elevationList), numel(azimuthList));
meanErrorGrid   = nan(numel(elevationList), numel(azimuthList));

fprintf('\n============================================================\n');
fprintf('PHASE C.1 - ONE-VIEW DIMENSION-OPTIMIZED SWEEP\n');
fprintf('Azimuth   = %s\n', mat2str(azimuthList));
fprintf('Elevation = %s\n', mat2str(elevationList));
fprintf('Fixed baseline = %.2f | HFOV = %.1f | Range = %.1f\n', baselineVal, hfovDegVal, rangeVal);
fprintf('Total trials = %d\n', numel(azimuthList) * numel(elevationList));
fprintf('============================================================\n\n');

caseCounter = 0;
for iAz = 1:numel(azimuthList)
    azimuthDeg = azimuthList(iAz);

    for iEl = 1:numel(elevationList)
        elevationDeg = elevationList(iEl);
        caseCounter = caseCounter + 1;

        fprintf('Case %d/%d: az = %d | el = %d\n', ...
            caseCounter, numel(azimuthList) * numel(elevationList), azimuthDeg, elevationDeg);

        try
            [renderData, rig, sceneInfo] = render_jason_ship_stereo( ... %#ok<NASGU,ASGLU>
                'ShipScript', 'oil_tanker_generate_Tanker.m', ...
                'AzimuthDeg', azimuthDeg, ...
                'ElevationDeg', elevationDeg, ...
                'Range', rangeVal, ...
                'Baseline', baselineVal, ...
                'ImageSize', imageSizeVal, ...
                'HFOVDeg', hfovDegVal, ...
                'MakeFigure', false, ...
                'Verbose', false, ...
                'SaveImages', false);

            recon = reconstruct_pointcloud(renderData, rig, ...
                'DisparityRange', disparityRangeVal, ...
                'MakeFigure', false, ...
                'Verbose', false, ...
                'SavePointCloud', false, ...
                'MaxDepth', maxDepthVal, ...
                'UseAdaptiveHistEq', useAdaptiveHistEqVal, ...
                'MinValidDisparity', minValidDisparityVal, ...
                'BackgroundThreshold', backgroundThresholdVal);

            metrics = compute_gt_metrics_jason_ship(recon, ...
                'CropX', cropXVal, ...
                'CropY', cropYVal, ...
                'CropZ', cropZVal, ...
                'GTMeshSpacing', gtMeshSpacingVal, ...
                'MaxEvalPoints', maxEvalPointsVal, ...
                'ChunkSizeNN', chunkSizeNNVal, ...
                'ChunkSizeMesh', chunkSizeMeshVal, ...
                'TriangleCandidates', triangleCandidatesVal, ...
                'MakeFigure', false, ...
                'Verbose', false);

            xyz = local_extract_cropped_xyz(recon, cropXVal, cropYVal, cropZVal);
            stats = local_compute_dimension_stats( ...
                xyz, gtDims, bootstrapSamples, maxBootstrapPoints, clearanceQuantile, safetyMargin);

            row.azimuth_deg = azimuthDeg;
            row.elevation_deg = elevationDeg;
            row.range = rangeVal;
            row.hfov_deg = hfovDegVal;
            row.baseline = baselineVal;

            row.length_est = stats.length_est;
            row.width_est  = stats.width_est;
            row.height_est = stats.height_est;

            row.length_rel_error_pct = stats.length_rel_error_pct;
            row.width_rel_error_pct  = stats.width_rel_error_pct;
            row.height_rel_error_pct = stats.height_rel_error_pct;
            row.mean_dimension_error_pct = stats.mean_dimension_error_pct;
            row.required_clearance_99 = stats.required_clearance_99;

            row.covered_gt_area_fraction_020 = metrics.covered_gt_area_fraction_020;
            row.mesh_surface_one_sided_rmse = metrics.mesh_surface_one_sided_rmse;
            row.centroid_symmetric_rmse = safe_get(metrics, 'centroid_symmetric_rmse');
            row.raw_point_count = safe_get(metrics, 'raw_point_count');
            row.cropped_point_count = safe_get(metrics, 'cropped_point_count');
            row.keep_fraction = safe_get(metrics, 'keep_fraction');

            rows = [rows; row]; %#ok<AGROW>
            heightErrorGrid(iEl, iAz) = row.height_rel_error_pct;
            meanErrorGrid(iEl, iAz)   = row.mean_dimension_error_pct;

            fprintf('    Height err = %.3f%% | Length err = %.3f%% | Width err = %.3f%% | Mean err = %.3f%% | Clearance99 = %.4f\n', ...
                row.height_rel_error_pct, row.length_rel_error_pct, row.width_rel_error_pct, ...
                row.mean_dimension_error_pct, row.required_clearance_99);

        catch ME
            warning('Case failed at az=%d el=%d | %s', azimuthDeg, elevationDeg, ME.message);
        end

        close all force;
        drawnow;
    end
end

if isempty(rows)
    error('All Phase C.1 dimension-optimized one-view cases failed.');
end

allTable = struct2table(rows);
allCsv = fullfile(resultsDir, ['phaseC1_one_view_dimensional_all_results_' timestampTag '.csv']);
writetable(allTable, allCsv);

[~, idxBestHeight] = min(allTable.height_rel_error_pct);
[~, idxBestLength] = min(allTable.length_rel_error_pct);
[~, idxBestWidth]  = min(allTable.width_rel_error_pct);
[~, idxBestMean]   = min(allTable.mean_dimension_error_pct);
[~, idxBestCov20]  = max(allTable.covered_gt_area_fraction_020);

winnerRows = [ ...
    allTable(idxBestHeight,:); ...
    allTable(idxBestLength,:); ...
    allTable(idxBestWidth,:); ...
    allTable(idxBestMean,:); ...
    allTable(idxBestCov20,:) ...
    ];
winnerRows.objective = [ ...
    "best_height_error"; ...
    "best_length_error"; ...
    "best_width_error"; ...
    "best_mean_dimension_error"; ...
    "best_gt_coverage_020" ...
    ];
winnerRows = movevars(winnerRows, 'objective', 'Before', 1);

winnerCsv = fullfile(resultsDir, ['phaseC1_one_view_dimensional_winners_' timestampTag '.csv']);
writetable(winnerRows, winnerCsv);

local_save_case_pointcloud( ...
    winnerRows(strcmp(winnerRows.objective, "best_height_error"),:), ...
    pcDir, 'best_height_case', ...
    disparityRangeVal, maxDepthVal, minValidDisparityVal, ...
    backgroundThresholdVal, useAdaptiveHistEqVal, ...
    cropXVal, cropYVal, cropZVal, imageSizeVal);

local_save_case_pointcloud( ...
    winnerRows(strcmp(winnerRows.objective, "best_mean_dimension_error"),:), ...
    pcDir, 'best_mean_case', ...
    disparityRangeVal, maxDepthVal, minValidDisparityVal, ...
    backgroundThresholdVal, useAdaptiveHistEqVal, ...
    cropXVal, cropYVal, cropZVal, imageSizeVal);

heightHeatmapPath = fullfile(plotsDir, ['phaseC1_one_view_dimensional_height_error_heatmap_' timestampTag '.png']);
meanHeatmapPath   = fullfile(plotsDir, ['phaseC1_one_view_dimensional_mean_error_heatmap_' timestampTag '.png']);

local_plot_heatmap(azimuthList, elevationList, heightErrorGrid, ...
    'Height Error (%)', 'Phase C.1 One-View Height Error', heightHeatmapPath);

local_plot_heatmap(azimuthList, elevationList, meanErrorGrid, ...
    'Mean Dimension Error (%)', 'Phase C.1 One-View Mean Dimension Error', meanHeatmapPath);

fprintf('\n============================================================\n');
fprintf('PHASE C.1 DIMENSION-OPTIMIZED SWEEP COMPLETE\n');
fprintf('All results CSV:\n%s\n\n', allCsv);
fprintf('Winner summary CSV:\n%s\n\n', winnerCsv);
fprintf('Height-error heatmap:\n%s\n\n', heightHeatmapPath);
fprintf('Mean-error heatmap:\n%s\n\n', meanHeatmapPath);
fprintf('Point clouds saved in:\n%s\n\n', pcDir);
disp(winnerRows(:, {'objective','azimuth_deg','elevation_deg', ...
    'height_rel_error_pct','length_rel_error_pct','width_rel_error_pct', ...
    'mean_dimension_error_pct','required_clearance_99','covered_gt_area_fraction_020'}));
fprintf('============================================================\n\n');

function v = safe_get(s, fieldName)
    if isfield(s, fieldName), v = s.(fieldName); else, v = NaN; end
end

function xyz = local_extract_cropped_xyz(recon, cropX, cropY, cropZ)
    xyz = [];
    if isfield(recon, 'cropped_xyz') && ~isempty(recon.cropped_xyz)
        xyz = recon.cropped_xyz;
    elseif isfield(recon, 'xyz_world') && ~isempty(recon.xyz_world)
        xyz = recon.xyz_world;
    elseif isfield(recon, 'points_xyz') && ~isempty(recon.points_xyz)
        xyz = recon.points_xyz;
    elseif isfield(recon, 'points') && ~isempty(recon.points)
        xyz = recon.points;
    elseif isfield(recon, 'ptCloud')
        xyz = recon.ptCloud.Location;
        xyz = reshape(xyz, [], 3);
    end
    if isempty(xyz), error('Could not extract XYZ points from reconstruction struct.'); end
    xyz = double(xyz);
    xyz = xyz(all(isfinite(xyz),2), :);
    keep = xyz(:,1) >= cropX(1) & xyz(:,1) <= cropX(2) & ...
           xyz(:,2) >= cropY(1) & xyz(:,2) <= cropY(2) & ...
           xyz(:,3) >= cropZ(1) & xyz(:,3) <= cropZ(2);
    xyz = xyz(keep, :);
    if isempty(xyz), error('No cropped points available after applying crop box.'); end
end

function stats = local_compute_dimension_stats(xyz, gtDims, bootstrapSamples, maxBootstrapPoints, clearanceQuantile, safetyMargin)
    mins = min(xyz, [], 1); maxs = max(xyz, [], 1); dims = maxs - mins;
    stats.length_est = dims(1); stats.width_est = dims(2); stats.height_est = dims(3);
    stats.length_rel_error_pct = 100 * abs(stats.length_est - gtDims.length) / gtDims.length;
    stats.width_rel_error_pct  = 100 * abs(stats.width_est  - gtDims.width)  / gtDims.width;
    stats.height_rel_error_pct = 100 * abs(stats.height_est - gtDims.height) / gtDims.height;
    stats.mean_dimension_error_pct = mean([stats.length_rel_error_pct, stats.width_rel_error_pct, stats.height_rel_error_pct]);
    n = size(xyz,1);
    if n > maxBootstrapPoints
        rng(1); idx = randperm(n, maxBootstrapPoints); xyzBootBase = xyz(idx, :);
    else
        xyzBootBase = xyz;
    end
    nBoot = size(xyzBootBase,1); bootHeights = nan(bootstrapSamples,1);
    for b = 1:bootstrapSamples
        pick = randi(nBoot, [nBoot, 1]); xb = xyzBootBase(pick, :);
        bootHeights(b) = max(xb(:,3)) - min(xb(:,3));
    end
    upperH = prctile(bootHeights, clearanceQuantile);
    stats.required_clearance_99 = upperH + safetyMargin;
end

function local_plot_heatmap(azimuthList, elevationList, metricGrid, cbarLabel, plotTitle, savePath)
    fig = figure('Color','w','Position',[100 100 860 560]);
    imagesc(azimuthList, elevationList, metricGrid);
    set(gca, 'YDir', 'normal', 'FontSize', 12, 'XColor', 'k', 'YColor', 'k', 'LineWidth', 1.0);
    xlabel('Azimuth (deg)', 'Color', 'k', 'FontWeight', 'bold');
    ylabel('Elevation (deg)', 'Color', 'k', 'FontWeight', 'bold');
    title(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
    colormap(parula(256));
    cb = colorbar; cb.Label.String = cbarLabel; cb.Color = 'k';
    axis tight;
    exportgraphics(fig, savePath, 'Resolution', 300);
    close(fig);
end

function local_save_case_pointcloud(caseRow, pcDir, tag, disparityRangeVal, maxDepthVal, minValidDisparityVal, backgroundThresholdVal, useAdaptiveHistEqVal, cropXVal, cropYVal, cropZVal, imageSizeVal)
    azimuthDeg = caseRow.azimuth_deg;
    elevationDeg = caseRow.elevation_deg;
    rangeVal = caseRow.range;
    baselineVal = caseRow.baseline;
    hfovDegVal = caseRow.hfov_deg;
    [renderData, rig, sceneInfo] = render_jason_ship_stereo( ... %#ok<NASGU,ASGLU>
        'ShipScript', 'oil_tanker_generate_Tanker.m', ...
        'AzimuthDeg', azimuthDeg, ...
        'ElevationDeg', elevationDeg, ...
        'Range', rangeVal, ...
        'Baseline', baselineVal, ...
        'ImageSize', imageSizeVal, ...
        'HFOVDeg', hfovDegVal, ...
        'MakeFigure', false, ...
        'Verbose', false, ...
        'SaveImages', false);
    recon = reconstruct_pointcloud(renderData, rig, ...
        'DisparityRange', disparityRangeVal, ...
        'MakeFigure', false, ...
        'Verbose', false, ...
        'SavePointCloud', false, ...
        'MaxDepth', maxDepthVal, ...
        'UseAdaptiveHistEq', useAdaptiveHistEqVal, ...
        'MinValidDisparity', minValidDisparityVal, ...
        'BackgroundThreshold', backgroundThresholdVal);
    xyz = local_extract_cropped_xyz(recon, cropXVal, cropYVal, cropZVal);
    pcwrite(pointCloud(xyz), fullfile(pcDir, [tag '.ply']));
end
