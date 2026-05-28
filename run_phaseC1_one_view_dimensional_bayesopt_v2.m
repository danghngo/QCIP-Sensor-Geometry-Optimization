function run_phaseC1_one_view_dimensional_bayesopt_v2()
% run_phaseC1_one_view_dimensional_bayesopt_v2
%
% Bayesian optimization driver for Phase C.1 one-view dimensional search.
% This version is wired to the Jason tanker evaluation logic and saves a
% colored point cloud for the overall best mean-dimension winner.
%
% Overnight recommendation:
%   - 10 seeds
%   - 25 random initial evaluations per seed
%   - 125 total evaluations per seed
%
% Search space:
%   azimuth   = 0:1:359
%   elevation = 5:1:35
%   baseline  = 0.08:0.02:0.32
%   HFOV      = 16:2:32
%   range     = 18:1:30   (widen to 36 later if desired)

clear; clc;

cfg = local_default_config();
local_ensure_dir(cfg.output_root);
seed_list_to_run = local_resolve_seed_list(cfg.seed_list);

fprintf('\n============================================================\n');
fprintf('Phase C.1 one-view dimensional BO driver (v2)\n');
fprintf('Output root: %s\n', cfg.output_root);
fprintf('Seeds to run: %s\n', mat2str(seed_list_to_run));
fprintf('Max evaluations per seed: %d\n', cfg.max_objective_evaluations);
fprintf('Num seed points: %d\n', cfg.num_seed_points);
fprintf('Acquisition: %s\n', cfg.acquisition_name);
fprintf('============================================================\n\n');

all_seed_summaries = table();

for iSeed = 1:numel(seed_list_to_run)
    seed = seed_list_to_run(iSeed);
    fprintf('\n------------------------------------------------------------\n');
    fprintf('Running BO seed %d (%d of %d)\n', seed, iSeed, numel(seed_list_to_run));
    fprintf('------------------------------------------------------------\n');

    rng(seed, 'twister');

    seed_dir = fullfile(cfg.output_root, sprintf('seed_%02d', seed));
    local_ensure_dir(seed_dir);

    eval_rows = table();

    vars = [ ...
        optimizableVariable('az_idx',       [1, numel(cfg.az_grid)],       'Type', 'integer'), ...
        optimizableVariable('el_idx',       [1, numel(cfg.el_grid)],       'Type', 'integer'), ...
        optimizableVariable('baseline_idx', [1, numel(cfg.baseline_grid)], 'Type', 'integer'), ...
        optimizableVariable('hfov_idx',     [1, numel(cfg.hfov_grid)],     'Type', 'integer'), ...
        optimizableVariable('range_idx',    [1, numel(cfg.range_grid)],    'Type', 'integer')  ...
    ];

    save_file_name = fullfile(seed_dir, sprintf('phaseC1_one_view_dimensional_bo_workspace_seed_%02d.mat', seed));

    results = bayesopt(@objective_fcn, vars, ...
        'Verbose', 1, ...
        'IsObjectiveDeterministic', true, ...
        'MaxObjectiveEvaluations', cfg.max_objective_evaluations, ...
        'NumSeedPoints', cfg.num_seed_points, ...
        'AcquisitionFunctionName', cfg.acquisition_name, ...
        'PlotFcn', {}, ...
        'UseParallel', false, ...
        'SaveFileName', save_file_name);

    eval_log_csv = fullfile(seed_dir, sprintf('phaseC1_one_view_dimensional_bo_eval_log_seed_%02d_%s.csv', seed, cfg.timestamp));
    if ~isempty(eval_rows)
        writetable(eval_rows, eval_log_csv);
    end

    best_candidate = local_decode_candidate(results.XAtMinObjective, cfg);
    best_metrics   = phaseC1_one_view_dimensional_bo_candidate_adapter_v2(best_candidate, cfg);
    [best_obj, best_breakdown] = local_compute_objective(best_metrics, cfg);

    best_summary = struct2table(local_pack_summary_row(seed, best_candidate, best_metrics, best_obj, best_breakdown));
    best_summary_csv = fullfile(seed_dir, sprintf('phaseC1_one_view_dimensional_bo_best_summary_seed_%02d_%s.csv', seed, cfg.timestamp));
    writetable(best_summary, best_summary_csv);
    all_seed_summaries = [all_seed_summaries; best_summary]; %#ok<AGROW>

    fprintf('Seed %d best objective: %.6f\n', seed, best_obj);
    fprintf('Best candidate: az=%g, el=%g, baseline=%.3f, hfov=%g, range=%g\n', ...
        best_candidate.azimuth_deg, best_candidate.elevation_deg, best_candidate.baseline, ...
        best_candidate.hfov_deg, best_candidate.range);
    fprintf('Best metrics: mean=%.4f%%, width=%.4f%%, height=%.4f%%, length=%.4f%%, gt020=%.4f\n', ...
        best_metrics.mean_dimension_error_pct, best_metrics.width_rel_error_pct, ...
        best_metrics.height_rel_error_pct, best_metrics.length_rel_error_pct, ...
        best_metrics.covered_gt_area_fraction_020);

    % Save a colored point cloud for each seed's best case.
    local_save_case_pointcloud(best_candidate, cfg, seed_dir, sprintf('seed_%02d_best_mean_case', seed));
end

if ~isempty(all_seed_summaries)
    combined_csv = fullfile(cfg.output_root, sprintf('phaseC1_one_view_dimensional_bo_all_seed_summaries_%s.csv', cfg.timestamp));
    writetable(all_seed_summaries, combined_csv);

    [~, idx_best_overall] = min(all_seed_summaries.objective_loss);
    overall_best = all_seed_summaries(idx_best_overall, :);
    overall_best_csv = fullfile(cfg.output_root, sprintf('phaseC1_one_view_dimensional_bo_overall_best_%s.csv', cfg.timestamp));
    writetable(overall_best, overall_best_csv);

    best_candidate = struct( ...
        'azimuth_deg', overall_best.azimuth_deg, ...
        'elevation_deg', overall_best.elevation_deg, ...
        'baseline', overall_best.baseline, ...
        'hfov_deg', overall_best.hfov_deg, ...
        'range', overall_best.range);

    local_save_case_pointcloud(best_candidate, cfg, cfg.output_root, 'overall_best_mean_case');
end

fprintf('\nDone. Output root:\n%s\n', cfg.output_root);

    function objective_loss = objective_fcn(X)
        candidate = local_decode_candidate(X, cfg);
        status = "ok";
        failure_message = "";

        try
            metrics = phaseC1_one_view_dimensional_bo_candidate_adapter_v2(candidate, cfg);
            [objective_loss, breakdown] = local_compute_objective(metrics, cfg);
        catch ME
            metrics = local_failure_metrics();
            [objective_loss, breakdown] = local_compute_objective(metrics, cfg);
            status = "error";
            failure_message = string(ME.message);
        end

        row = struct2table(local_pack_eval_row(seed, candidate, metrics, objective_loss, breakdown, status, failure_message));
        eval_rows = [eval_rows; row]; %#ok<AGROW>

        fprintf('seed=%d | az=%3g el=%2g b=%.3f hfov=%2g r=%2g | obj=%10.4f | mean=%8.4f width=%8.4f gt020=%6.4f | %s\n', ...
            seed, candidate.azimuth_deg, candidate.elevation_deg, candidate.baseline, ...
            candidate.hfov_deg, candidate.range, objective_loss, ...
            metrics.mean_dimension_error_pct, metrics.width_rel_error_pct, ...
            metrics.covered_gt_area_fraction_020, status);
    end
end

function cfg = local_default_config()
    cfg.timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    cfg.seed_list = 1:10;
    cfg.max_objective_evaluations = 125;
    cfg.num_seed_points = 25;
    cfg.acquisition_name = 'expected-improvement-plus';

    cfg.az_grid       = 0:359;
    cfg.el_grid       = 5:35;
    cfg.baseline_grid = 0.08:0.02:0.32;
    cfg.hfov_grid     = 16:2:32;
    cfg.range_grid    = 18:1:30;

    cfg.weights.mean_dim = 0.55;
    cfg.weights.width    = 0.30;
    cfg.weights.max_dim  = 0.15;
    cfg.min_gt020 = 0.45;
    cfg.max_clearance99 = 1.80;
    cfg.penalty.coverage  = 25.0;
    cfg.penalty.clearance = 5.0;
    cfg.penalty.failure   = 1.0e6;

    cfg.this_dir = fileparts(mfilename('fullpath'));
    if isempty(cfg.this_dir), cfg.this_dir = pwd; end
    cfg.output_root = fullfile(cfg.this_dir, 'outputs', ['phaseC1_one_view_dimensional_BO_' cfg.timestamp]);

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
    cfg.gt_dims.width  = 1.2;
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

function candidate = local_decode_candidate(X, cfg)
    candidate.azimuth_deg   = cfg.az_grid(X.az_idx);
    candidate.elevation_deg = cfg.el_grid(X.el_idx);
    candidate.baseline      = cfg.baseline_grid(X.baseline_idx);
    candidate.hfov_deg      = cfg.hfov_grid(X.hfov_idx);
    candidate.range         = cfg.range_grid(X.range_idx);

    candidate.az_idx       = X.az_idx;
    candidate.el_idx       = X.el_idx;
    candidate.baseline_idx = X.baseline_idx;
    candidate.hfov_idx     = X.hfov_idx;
    candidate.range_idx    = X.range_idx;
end

function [objective_loss, breakdown] = local_compute_objective(metrics, cfg)
    max_dim_error_pct = max([metrics.height_rel_error_pct, metrics.length_rel_error_pct, metrics.width_rel_error_pct]);
    coverage_shortfall = max(0.0, cfg.min_gt020 - metrics.covered_gt_area_fraction_020);
    clearance_excess   = max(0.0, metrics.required_clearance_99 - cfg.max_clearance99);

    objective_loss = ...
        cfg.weights.mean_dim * metrics.mean_dimension_error_pct + ...
        cfg.weights.width    * metrics.width_rel_error_pct + ...
        cfg.weights.max_dim  * max_dim_error_pct + ...
        cfg.penalty.coverage  * (coverage_shortfall^2) + ...
        cfg.penalty.clearance * (clearance_excess^2);

    if isfield(metrics, 'failure_flag') && metrics.failure_flag
        objective_loss = objective_loss + cfg.penalty.failure;
    end

    breakdown.max_dim_error_pct = max_dim_error_pct;
    breakdown.coverage_shortfall = coverage_shortfall;
    breakdown.clearance_excess = clearance_excess;
end

function row = local_pack_eval_row(seed, candidate, metrics, objective_loss, breakdown, status, failure_message)
    row.seed = seed;
    row.azimuth_deg = candidate.azimuth_deg;
    row.elevation_deg = candidate.elevation_deg;
    row.baseline = candidate.baseline;
    row.hfov_deg = candidate.hfov_deg;
    row.range = candidate.range;

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
    row.status = string(status);
    row.failure_message = string(failure_message);
end

function row = local_pack_summary_row(seed, candidate, metrics, objective_loss, breakdown)
    row = local_pack_eval_row(seed, candidate, metrics, objective_loss, breakdown, "ok", "");
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
end

function local_save_case_pointcloud(candidate, cfg, out_dir, tag)
    [renderData, rig] = render_jason_ship_stereo( ...
        'ShipScript', cfg.ship_script, ...
        'AzimuthDeg', candidate.azimuth_deg, ...
        'ElevationDeg', candidate.elevation_deg, ...
        'Range', candidate.range, ...
        'Baseline', candidate.baseline, ...
        'ImageSize', cfg.image_size, ...
        'HFOVDeg', candidate.hfov_deg, ...
        'MakeFigure', false, ...
        'Verbose', false, ...
        'SaveImages', false);

    recon = reconstruct_pointcloud(renderData, rig, ...
        'DisparityRange', cfg.disparity_range, ...
        'MakeFigure', false, ...
        'Verbose', false, ...
        'SavePointCloud', false, ...
        'MaxDepth', cfg.max_depth, ...
        'UseAdaptiveHistEq', cfg.use_adaptive_hist_eq, ...
        'MinValidDisparity', cfg.min_valid_disparity, ...
        'BackgroundThreshold', cfg.background_threshold);

    [xyz, rgb] = local_extract_cropped_cloud(recon, cfg.crop_x, cfg.crop_y, cfg.crop_z);
    if isempty(rgb)
        pc = pointCloud(xyz);
    else
        pc = pointCloud(xyz, 'Color', rgb);
    end
    pcwrite(pc, fullfile(out_dir, [tag '.ply']));
end

function [xyz, rgb] = local_extract_cropped_cloud(recon, cropX, cropY, cropZ)
    xyz = [];
    rgb = [];

    % Best case: existing colored pointCloud object.
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
    if isfield(s, field_name)
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
