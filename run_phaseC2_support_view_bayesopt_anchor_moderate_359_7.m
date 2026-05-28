function run_phaseC2_support_view_bayesopt_anchor_moderate_359_7()
% run_phaseC2_support_view_bayesopt_anchor_moderate_359_7
%
% Phase C.2 corrective support-view Bayesian optimization.
% Fixed first-view anchor = moderate-error anchor 359/7
% Approximate one-view mean error = 5.07%
% This run keeps the rig fixed to the first-view anchor and only
% optimizes the SECOND view azimuth/elevation. That keeps the study focused
% on corrective support geometry.
%
% Search space for second view:
%   azimuth   = 0:1:359
%   elevation = 5:1:35
%
% BO settings:
%   10 seeds
%   25 random initial evaluations per seed
%   125 total evaluations per seed

clear; clc;

cfg = local_default_config();
local_ensure_dir(cfg.output_root);
seed_list_to_run = local_resolve_seed_list(cfg.seed_list);

addpath(genpath(cfg.this_dir));
rehash;

fprintf('\n============================================================\n');
fprintf('Phase C.2 support-view BO driver (moderate-error anchor 359/7)\n');
fprintf('Output root: %s\n', cfg.output_root);
fprintf('Fixed first view: az=%g, el=%g, baseline=%.3f, hfov=%g, range=%g\n', ...
    cfg.first_view.azimuth_deg, cfg.first_view.elevation_deg, cfg.first_view.baseline, ...
    cfg.first_view.hfov_deg, cfg.first_view.range);
fprintf('Second-view azimuth grid: %s\n', local_range_string(cfg.second_az_grid));
fprintf('Second-view elevation grid: %s\n', local_range_string(cfg.second_el_grid));
fprintf('Seeds to run: %s\n', mat2str(seed_list_to_run));
fprintf('Max evaluations per seed: %d\n', cfg.max_objective_evaluations);
fprintf('Num seed points: %d\n', cfg.num_seed_points);
fprintf('Acquisition: %s\n', cfg.acquisition_name);
fprintf('============================================================\n\n');

fprintf('Building fixed first-view anchor once...\n');
[anchor_recon, anchor_metrics, anchor_cloud] = local_build_first_view_anchor(cfg);
anchor_summary = struct2table(local_pack_anchor_row(cfg.first_view, anchor_metrics));
anchor_summary_csv = fullfile(cfg.output_root, sprintf('phaseC2_support_view_anchor_summary_%s.csv', cfg.timestamp));
writetable(anchor_summary, anchor_summary_csv);
local_save_cloud(anchor_cloud.xyz, anchor_cloud.rgb, fullfile(cfg.output_root, 'anchor_first_view_case.ply'));

fprintf('Anchor metrics: mean=%.4f%% | height=%.4f%% | length=%.4f%% | width=%.4f%% | gt020=%.4f\n\n', ...
    anchor_metrics.mean_dimension_error_pct, anchor_metrics.height_rel_error_pct, ...
    anchor_metrics.length_rel_error_pct, anchor_metrics.width_rel_error_pct, ...
    anchor_metrics.covered_gt_area_fraction_020);

all_seed_summaries = table();

% Shared state for the nested objective function.
seed = NaN;
eval_rows = table();

for iSeed = 1:numel(seed_list_to_run)
    seed = seed_list_to_run(iSeed);
    fprintf('\n------------------------------------------------------------\n');
    fprintf('Running support-view BO seed %d (%d of %d)\n', seed, iSeed, numel(seed_list_to_run));
    fprintf('------------------------------------------------------------\n');

    rng(seed, 'twister');
    seed_dir = fullfile(cfg.output_root, sprintf('seed_%02d', seed));
    local_ensure_dir(seed_dir);
    eval_rows = table();

    vars = [ ...
        optimizableVariable('second_az_idx', [1, numel(cfg.second_az_grid)], 'Type', 'integer'), ...
        optimizableVariable('second_el_idx', [1, numel(cfg.second_el_grid)], 'Type', 'integer')  ...
    ];

    save_file_name = fullfile(seed_dir, sprintf('phaseC2_support_view_bo_workspace_seed_%02d.mat', seed));

    results = bayesopt(@objective_fcn, vars, ...
        'Verbose', 1, ...
        'IsObjectiveDeterministic', true, ...
        'MaxObjectiveEvaluations', cfg.max_objective_evaluations, ...
        'NumSeedPoints', cfg.num_seed_points, ...
        'AcquisitionFunctionName', cfg.acquisition_name, ...
        'PlotFcn', {}, ...
        'UseParallel', false, ...
        'SaveFileName', save_file_name);

    eval_log_csv = fullfile(seed_dir, sprintf('phaseC2_support_view_bo_eval_log_seed_%02d_%s.csv', seed, cfg.timestamp));
    if ~isempty(eval_rows)
        writetable(eval_rows, eval_log_csv);
    end

    best_second = local_decode_second_candidate(results.XAtMinObjective, cfg);
    best_metrics = phaseC2_support_view_bo_candidate_adapter_v1(cfg.first_view, best_second, anchor_recon, cfg);
    [best_obj, best_breakdown] = local_compute_objective(best_metrics, anchor_metrics, cfg);

    best_summary = struct2table(local_pack_summary_row(seed, cfg.first_view, best_second, best_metrics, anchor_metrics, best_obj, best_breakdown));
    best_summary_csv = fullfile(seed_dir, sprintf('phaseC2_support_view_bo_best_summary_seed_%02d_%s.csv', seed, cfg.timestamp));
    writetable(best_summary, best_summary_csv);
    all_seed_summaries = [all_seed_summaries; best_summary]; %#ok<AGROW>

    fprintf('Seed %d best objective: %.6f\n', seed, best_obj);
    fprintf('Best second view: az=%g, el=%g\n', best_second.azimuth_deg, best_second.elevation_deg);
    fprintf('Best merged metrics: mean=%.4f%%, width=%.4f%%, height=%.4f%%, length=%.4f%%, gt020=%.4f, dMean=%.4f\n', ...
        best_metrics.mean_dimension_error_pct, best_metrics.width_rel_error_pct, ...
        best_metrics.height_rel_error_pct, best_metrics.length_rel_error_pct, ...
        best_metrics.covered_gt_area_fraction_020, best_breakdown.delta_mean_pct);

    local_save_two_view_pointcloud(cfg.first_view, best_second, anchor_recon, cfg, seed_dir, sprintf('seed_%02d_best_support_case', seed));
end

if ~isempty(all_seed_summaries)
    combined_csv = fullfile(cfg.output_root, sprintf('phaseC2_support_view_bo_all_seed_summaries_%s.csv', cfg.timestamp));
    writetable(all_seed_summaries, combined_csv);

    [~, idx_best_overall] = min(all_seed_summaries.objective_loss);
    overall_best = all_seed_summaries(idx_best_overall, :);
    overall_best_csv = fullfile(cfg.output_root, sprintf('phaseC2_support_view_bo_overall_best_%s.csv', cfg.timestamp));
    writetable(overall_best, overall_best_csv);

    best_second.azimuth_deg = overall_best.second_azimuth_deg;
    best_second.elevation_deg = overall_best.second_elevation_deg;
    best_second.baseline = cfg.first_view.baseline;
    best_second.hfov_deg = cfg.first_view.hfov_deg;
    best_second.range = cfg.first_view.range;
    local_save_two_view_pointcloud(cfg.first_view, best_second, anchor_recon, cfg, cfg.output_root, 'overall_best_support_case');
end

fprintf('\nDone. Output root:\n%s\n', cfg.output_root);

    function objective_loss = objective_fcn(X)
        second_view = local_decode_second_candidate(X, cfg);
        status = "ok";
        failure_message = "";

        try
            metrics = phaseC2_support_view_bo_candidate_adapter_v1(cfg.first_view, second_view, anchor_recon, cfg);
            [objective_loss, breakdown] = local_compute_objective(metrics, anchor_metrics, cfg);
        catch ME
            metrics = local_failure_metrics();
            [objective_loss, breakdown] = local_compute_objective(metrics, anchor_metrics, cfg);
            status = "error";
            failure_message = string(ME.message);
        end

        row = struct2table(local_pack_eval_row(seed, cfg.first_view, second_view, metrics, anchor_metrics, objective_loss, breakdown, status, failure_message));
        eval_rows = [eval_rows; row]; %#ok<AGROW>

        fprintf('seed=%d | 1st=(%3g,%2g) 2nd=(%3g,%2g) | obj=%10.4f | mean=%8.4f dMean=%8.4f width=%8.4f gt020=%6.4f | %s\n', ...
            seed, cfg.first_view.azimuth_deg, cfg.first_view.elevation_deg, second_view.azimuth_deg, second_view.elevation_deg, ...
            objective_loss, metrics.mean_dimension_error_pct, breakdown.delta_mean_pct, ...
            metrics.width_rel_error_pct, metrics.covered_gt_area_fraction_020, status);
    end
end
function cfg = local_default_config()
    cfg.timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    cfg.seed_list = 1:10;
    cfg.max_objective_evaluations = 125;
    cfg.num_seed_points = 25;
    cfg.acquisition_name = 'expected-improvement-plus';

    cfg.first_view.azimuth_deg = 359;
    cfg.first_view.elevation_deg = 7;
    cfg.first_view.baseline = 0.26;
    cfg.first_view.hfov_deg = 22;
    cfg.first_view.range = 23;

    cfg.second_az_grid = 0:359;
    cfg.second_el_grid = 5:35;

    cfg.weights.mean_dim = 0.55;
    cfg.weights.width = 0.30;
    cfg.weights.max_dim = 0.15;
    cfg.min_gt020 = 0.45;
    cfg.max_clearance99 = 1.80;
    cfg.penalty.coverage = 25.0;
    cfg.penalty.clearance = 5.0;
    cfg.penalty.failure = 1.0e6;
    cfg.penalty.no_improvement = 10.0;
    cfg.penalty.duplicate_view = 100.0;

    cfg.this_dir = fileparts(mfilename('fullpath'));
    if isempty(cfg.this_dir), cfg.this_dir = pwd; end
    cfg.output_root = fullfile(cfg.this_dir, 'outputs', ['phaseC2_support_view_BO_anchor_moderate_359_7_' cfg.timestamp]);

    cfg.ship_script = 'oil_tanker_generate_Tanker.m';
    cfg.image_size = [720 1280];
    cfg.disparity_range = [16 64];
    cfg.max_depth = 30;
    cfg.min_valid_disparity = 16;
    cfg.background_threshold = 245;
    cfg.use_adaptive_hist_eq = true;

    cfg.crop_x = [-0.2 8.8];
    cfg.crop_y = [-1.4 1.4];
    cfg.crop_z = [0.0 2.8];

    cfg.gt_dims.length = 8.4;
    cfg.gt_dims.width = 1.2;
    cfg.gt_dims.height = 1.52;

    cfg.bootstrap_samples = 1000;
    cfg.max_bootstrap_points = 20000;
    cfg.clearance_quantile = 99;
    cfg.safety_margin = 0.0;

    cfg.gt_mesh_spacing = 0.10;
    cfg.max_eval_points = 30000;
    cfg.chunk_size_nn = 300;
    cfg.chunk_size_mesh = 120;
    cfg.triangle_candidates = 12;
end

function seed_list_to_run = local_resolve_seed_list(seed_list)
    task_str = getenv('SLURM_ARRAY_TASK_ID');
    if ~isempty(task_str)
        task_idx = str2double(task_str);
        if isfinite(task_idx) && task_idx >= 1 && task_idx <= numel(seed_list)
            seed_list_to_run = seed_list(task_idx);
            return;
        end
    end
    seed_list_to_run = seed_list;
end

function second_view = local_decode_second_candidate(X, cfg)
    second_view.azimuth_deg = cfg.second_az_grid(X.second_az_idx);
    second_view.elevation_deg = cfg.second_el_grid(X.second_el_idx);
    second_view.baseline = cfg.first_view.baseline;
    second_view.hfov_deg = cfg.first_view.hfov_deg;
    second_view.range = cfg.first_view.range;
    second_view.second_az_idx = X.second_az_idx;
    second_view.second_el_idx = X.second_el_idx;
end

function [anchor_recon, anchor_metrics, anchor_cloud] = local_build_first_view_anchor(cfg)
    [renderData, rig] = render_jason_ship_stereo( ...
        'ShipScript', cfg.ship_script, ...
        'AzimuthDeg', cfg.first_view.azimuth_deg, ...
        'ElevationDeg', cfg.first_view.elevation_deg, ...
        'Range', cfg.first_view.range, ...
        'Baseline', cfg.first_view.baseline, ...
        'ImageSize', cfg.image_size, ...
        'HFOVDeg', cfg.first_view.hfov_deg, ...
        'MakeFigure', false, ...
        'Verbose', false, ...
        'SaveImages', false);

    anchor_recon = reconstruct_pointcloud(renderData, rig, ...
        'DisparityRange', cfg.disparity_range, ...
        'MakeFigure', false, ...
        'Verbose', false, ...
        'SavePointCloud', false, ...
        'MaxDepth', cfg.max_depth, ...
        'UseAdaptiveHistEq', cfg.use_adaptive_hist_eq, ...
        'MinValidDisparity', cfg.min_valid_disparity, ...
        'BackgroundThreshold', cfg.background_threshold);

    metrics = compute_gt_metrics_jason_ship(anchor_recon, ...
        'CropX', cfg.crop_x, ...
        'CropY', cfg.crop_y, ...
        'CropZ', cfg.crop_z, ...
        'GTMeshSpacing', cfg.gt_mesh_spacing, ...
        'MaxEvalPoints', cfg.max_eval_points, ...
        'ChunkSizeNN', cfg.chunk_size_nn, ...
        'ChunkSizeMesh', cfg.chunk_size_mesh, ...
        'TriangleCandidates', cfg.triangle_candidates, ...
        'MakeFigure', false, ...
        'Verbose', false);

    [xyz, rgb] = local_extract_cropped_cloud(anchor_recon, cfg.crop_x, cfg.crop_y, cfg.crop_z);
    stats = local_compute_dimension_stats(xyz, cfg.gt_dims, cfg.bootstrap_samples, cfg.max_bootstrap_points, cfg.clearance_quantile, cfg.safety_margin);

    anchor_metrics = local_metrics_from_stats_and_gt(stats, metrics);
    anchor_cloud.xyz = xyz;
    anchor_cloud.rgb = rgb;
end

function [objective_loss, breakdown] = local_compute_objective(metrics, anchor_metrics, cfg)
    max_dim_error_pct = max([metrics.height_rel_error_pct, metrics.length_rel_error_pct, metrics.width_rel_error_pct]);
    coverage_shortfall = max(0.0, cfg.min_gt020 - metrics.covered_gt_area_fraction_020);
    clearance_excess = max(0.0, metrics.required_clearance_99 - cfg.max_clearance99);
    delta_mean_pct = metrics.mean_dimension_error_pct - anchor_metrics.mean_dimension_error_pct;
    delta_width_pct = metrics.width_rel_error_pct - anchor_metrics.width_rel_error_pct;

    no_improvement_penalty = cfg.penalty.no_improvement * max(0.0, delta_mean_pct)^2;
    duplicate_penalty = 0.0;
    if isfield(metrics, 'duplicate_second_view') && metrics.duplicate_second_view
        duplicate_penalty = cfg.penalty.duplicate_view;
    end

    objective_loss = ...
        cfg.weights.mean_dim * metrics.mean_dimension_error_pct + ...
        cfg.weights.width * metrics.width_rel_error_pct + ...
        cfg.weights.max_dim * max_dim_error_pct + ...
        cfg.penalty.coverage * (coverage_shortfall^2) + ...
        cfg.penalty.clearance * (clearance_excess^2) + ...
        no_improvement_penalty + duplicate_penalty;

    if isfield(metrics, 'failure_flag') && metrics.failure_flag
        objective_loss = objective_loss + cfg.penalty.failure;
    end

    breakdown.max_dim_error_pct = max_dim_error_pct;
    breakdown.coverage_shortfall = coverage_shortfall;
    breakdown.clearance_excess = clearance_excess;
    breakdown.delta_mean_pct = delta_mean_pct;
    breakdown.delta_width_pct = delta_width_pct;
    breakdown.no_improvement_penalty = no_improvement_penalty;
    breakdown.duplicate_penalty = duplicate_penalty;
end

function row = local_pack_anchor_row(first_view, metrics)
    row.first_azimuth_deg = first_view.azimuth_deg;
    row.first_elevation_deg = first_view.elevation_deg;
    row.baseline = first_view.baseline;
    row.hfov_deg = first_view.hfov_deg;
    row.range = first_view.range;
    row.height_rel_error_pct = metrics.height_rel_error_pct;
    row.length_rel_error_pct = metrics.length_rel_error_pct;
    row.width_rel_error_pct = metrics.width_rel_error_pct;
    row.mean_dimension_error_pct = metrics.mean_dimension_error_pct;
    row.required_clearance_99 = metrics.required_clearance_99;
    row.covered_gt_area_fraction_020 = metrics.covered_gt_area_fraction_020;
    row.covered_gt_area_fraction_005 = local_getfield_default(metrics, 'covered_gt_area_fraction_005', NaN);
    row.covered_gt_area_fraction_010 = local_getfield_default(metrics, 'covered_gt_area_fraction_010', NaN);
    row.mesh_surface_one_sided_rmse = local_getfield_default(metrics, 'mesh_surface_one_sided_rmse', NaN);
    row.centroid_symmetric_rmse = local_getfield_default(metrics, 'centroid_symmetric_rmse', NaN);
end

function row = local_pack_eval_row(seed, first_view, second_view, metrics, anchor_metrics, objective_loss, breakdown, status, failure_message)
    row.seed = seed;
    row.first_azimuth_deg = first_view.azimuth_deg;
    row.first_elevation_deg = first_view.elevation_deg;
    row.second_azimuth_deg = second_view.azimuth_deg;
    row.second_elevation_deg = second_view.elevation_deg;
    row.baseline = first_view.baseline;
    row.hfov_deg = first_view.hfov_deg;
    row.range = first_view.range;

    row.objective_loss = objective_loss;
    row.height_rel_error_pct = metrics.height_rel_error_pct;
    row.length_rel_error_pct = metrics.length_rel_error_pct;
    row.width_rel_error_pct = metrics.width_rel_error_pct;
    row.mean_dimension_error_pct = metrics.mean_dimension_error_pct;
    row.max_dimension_error_pct = breakdown.max_dim_error_pct;
    row.required_clearance_99 = metrics.required_clearance_99;
    row.covered_gt_area_fraction_020 = metrics.covered_gt_area_fraction_020;
    row.covered_gt_area_fraction_005 = local_getfield_default(metrics, 'covered_gt_area_fraction_005', NaN);
    row.covered_gt_area_fraction_010 = local_getfield_default(metrics, 'covered_gt_area_fraction_010', NaN);
    row.mesh_surface_one_sided_rmse = local_getfield_default(metrics, 'mesh_surface_one_sided_rmse', NaN);
    row.centroid_symmetric_rmse = local_getfield_default(metrics, 'centroid_symmetric_rmse', NaN);
    row.raw_point_count = local_getfield_default(metrics, 'raw_point_count', NaN);
    row.cropped_point_count = local_getfield_default(metrics, 'cropped_point_count', NaN);
    row.keep_fraction = local_getfield_default(metrics, 'keep_fraction', NaN);

    row.anchor_mean_dimension_error_pct = anchor_metrics.mean_dimension_error_pct;
    row.anchor_width_rel_error_pct = anchor_metrics.width_rel_error_pct;
    row.delta_mean_dimension_error_pct = breakdown.delta_mean_pct;
    row.delta_width_rel_error_pct = breakdown.delta_width_pct;
    row.no_improvement_penalty = breakdown.no_improvement_penalty;
    row.duplicate_penalty = breakdown.duplicate_penalty;
    row.status = string(status);
    row.failure_message = string(failure_message);
end

function row = local_pack_summary_row(seed, first_view, second_view, metrics, anchor_metrics, objective_loss, breakdown)
    row = local_pack_eval_row(seed, first_view, second_view, metrics, anchor_metrics, objective_loss, breakdown, "ok", "");
end

function metrics = local_failure_metrics()
    big = 1.0e4;
    metrics.height_rel_error_pct = big;
    metrics.length_rel_error_pct = big;
    metrics.width_rel_error_pct = big;
    metrics.mean_dimension_error_pct = big;
    metrics.required_clearance_99 = big;
    metrics.covered_gt_area_fraction_020 = 0.0;
    metrics.covered_gt_area_fraction_005 = 0.0;
    metrics.covered_gt_area_fraction_010 = 0.0;
    metrics.mesh_surface_one_sided_rmse = NaN;
    metrics.centroid_symmetric_rmse = NaN;
    metrics.raw_point_count = 0;
    metrics.cropped_point_count = 0;
    metrics.keep_fraction = 0;
    metrics.failure_flag = true;
    metrics.failure_reason = "evaluation_failure";
    metrics.duplicate_second_view = false;
end

function local_save_two_view_pointcloud(first_view, second_view, anchor_recon, cfg, out_dir, tag)
    metrics = phaseC2_support_view_bo_candidate_adapter_v1(first_view, second_view, anchor_recon, cfg);
    if isfield(metrics, 'merged_xyz')
        local_save_cloud(metrics.merged_xyz, local_getfield_default(metrics, 'merged_rgb', []), fullfile(out_dir, [tag '.ply']));
    else
        error('Merged point cloud fields were not returned by the adapter.');
    end
end

function local_save_cloud(xyz, rgb, save_path)
    if isempty(rgb)
        pc = pointCloud(xyz);
    else
        pc = pointCloud(xyz, 'Color', rgb);
    end
    pcwrite(pc, save_path);
end

function metrics = local_metrics_from_stats_and_gt(stats, gt_metrics)
    metrics.length_est = stats.length_est;
    metrics.width_est = stats.width_est;
    metrics.height_est = stats.height_est;
    metrics.length_rel_error_pct = stats.length_rel_error_pct;
    metrics.width_rel_error_pct = stats.width_rel_error_pct;
    metrics.height_rel_error_pct = stats.height_rel_error_pct;
    metrics.mean_dimension_error_pct = stats.mean_dimension_error_pct;
    metrics.required_clearance_99 = stats.required_clearance_99;
    metrics.covered_gt_area_fraction_020 = local_getfield_default(gt_metrics, 'covered_gt_area_fraction_020', NaN);
    metrics.covered_gt_area_fraction_005 = local_getfield_default(gt_metrics, 'covered_gt_area_fraction_005', NaN);
    metrics.covered_gt_area_fraction_010 = local_getfield_default(gt_metrics, 'covered_gt_area_fraction_010', NaN);
    metrics.mesh_surface_one_sided_rmse = local_getfield_default(gt_metrics, 'mesh_surface_one_sided_rmse', NaN);
    metrics.centroid_symmetric_rmse = local_getfield_default(gt_metrics, 'centroid_symmetric_rmse', NaN);
    metrics.raw_point_count = local_getfield_default(gt_metrics, 'raw_point_count', NaN);
    metrics.cropped_point_count = local_getfield_default(gt_metrics, 'cropped_point_count', NaN);
    metrics.keep_fraction = local_getfield_default(gt_metrics, 'keep_fraction', NaN);
    metrics.failure_flag = false;
    metrics.duplicate_second_view = false;
end

function stats = local_compute_dimension_stats(xyz, gtDims, bootstrapSamples, maxBootstrapPoints, clearanceQuantile, safetyMargin)
    mins = min(xyz, [], 1);
    maxs = max(xyz, [], 1);
    dims = maxs - mins;

    stats.length_est = dims(1);
    stats.width_est = dims(2);
    stats.height_est = dims(3);
    stats.length_rel_error_pct = 100 * abs(stats.length_est - gtDims.length) / gtDims.length;
    stats.width_rel_error_pct = 100 * abs(stats.width_est - gtDims.width) / gtDims.width;
    stats.height_rel_error_pct = 100 * abs(stats.height_est - gtDims.height) / gtDims.height;
    stats.mean_dimension_error_pct = mean([stats.length_rel_error_pct, stats.width_rel_error_pct, stats.height_rel_error_pct]);

    n = size(xyz,1);
    if n > maxBootstrapPoints
        rng(1);
        idx = randperm(n, maxBootstrapPoints);
        xyzBootBase = xyz(idx, :);
    else
        xyzBootBase = xyz;
    end
    nBoot = size(xyzBootBase,1);
    bootHeights = nan(bootstrapSamples,1);
    for b = 1:bootstrapSamples
        pick = randi(nBoot, [nBoot, 1]);
        xb = xyzBootBase(pick, :);
        bootHeights(b) = max(xb(:,3)) - min(xb(:,3));
    end
    upperH = prctile(bootHeights, clearanceQuantile);
    stats.required_clearance_99 = upperH + safetyMargin;
end

function [xyz, rgb] = local_extract_cropped_cloud(recon, cropX, cropY, cropZ)
    xyz = [];
    rgb = [];

    if isfield(recon, 'ptCloud') && isa(recon.ptCloud, 'pointCloud')
        xyz = reshape(double(recon.ptCloud.Location), [], 3);
        if ~isempty(recon.ptCloud.Color)
            rgb = reshape(recon.ptCloud.Color, [], size(recon.ptCloud.Color, 3));
        end
    elseif isfield(recon, 'cropped_xyz') && ~isempty(recon.cropped_xyz)
        xyz = double(recon.cropped_xyz);
        if isfield(recon, 'cropped_rgb') && ~isempty(recon.cropped_rgb)
            rgb = recon.cropped_rgb;
        end
    elseif isfield(recon, 'xyz_world') && ~isempty(recon.xyz_world)
        xyz = double(recon.xyz_world);
        if isfield(recon, 'rgb') && ~isempty(recon.rgb)
            rgb = recon.rgb;
        elseif isfield(recon, 'points_rgb') && ~isempty(recon.points_rgb)
            rgb = recon.points_rgb;
        end
    elseif isfield(recon, 'points_xyz') && ~isempty(recon.points_xyz)
        xyz = double(recon.points_xyz);
        if isfield(recon, 'points_rgb') && ~isempty(recon.points_rgb)
            rgb = recon.points_rgb;
        end
    elseif isfield(recon, 'points') && ~isempty(recon.points)
        xyz = double(recon.points);
        if isfield(recon, 'colors') && ~isempty(recon.colors)
            rgb = recon.colors;
        elseif isfield(recon, 'rgb') && ~isempty(recon.rgb)
            rgb = recon.rgb;
        end
    end

    if isempty(xyz)
        error('Could not extract XYZ points from reconstruction struct.');
    end

    xyz = reshape(double(xyz), [], 3);
    xyz_valid = all(isfinite(xyz), 2);
    xyz = xyz(xyz_valid, :);

    if ~isempty(rgb)
        rgb = local_reshape_rgb(rgb);
        if size(rgb,1) == numel(xyz_valid)
            rgb = rgb(xyz_valid, :);
        elseif size(rgb,1) ~= size(xyz,1)
            rgb = [];
        end
    end

    keep = xyz(:,1) >= cropX(1) & xyz(:,1) <= cropX(2) & ...
           xyz(:,2) >= cropY(1) & xyz(:,2) <= cropY(2) & ...
           xyz(:,3) >= cropZ(1) & xyz(:,3) <= cropZ(2);

    xyz = xyz(keep, :);
    if ~isempty(rgb)
        rgb = rgb(keep, :);
        if isa(rgb, 'double') || isa(rgb, 'single')
            if max(rgb(:)) <= 1.0
                rgb = uint8(255 * rgb);
            else
                rgb = uint8(rgb);
            end
        else
            rgb = uint8(rgb);
        end
    end

    if isempty(xyz)
        error('No cropped points available after applying crop box.');
    end
end

function rgb = local_reshape_rgb(rgb)
    if isempty(rgb)
        return;
    end
    if ndims(rgb) == 3
        rgb = reshape(rgb, [], size(rgb,3));
    else
        rgb = reshape(rgb, [], size(rgb, ndims(rgb)));
    end
    if size(rgb,2) > 3
        rgb = rgb(:,1:3);
    elseif size(rgb,2) == 1
        rgb = repmat(rgb, 1, 3);
    end
end

function value = local_getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        value = s.(field_name);
    else
        value = default_value;
    end
end

function local_ensure_dir(d)
    if ~exist(d, 'dir')
        mkdir(d);
    end
end

function s = local_range_string(v)
    if numel(v) >= 2
        s = sprintf('%g:%g:%g', v(1), v(2)-v(1), v(end));
    else
        s = mat2str(v);
    end
end
