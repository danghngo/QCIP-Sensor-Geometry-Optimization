function dimResult = estimate_dimensions_with_ci_jason_ship(recon, varargin)
%ESTIMATE_DIMENSIONS_WITH_CI_JASON_SHIP Estimate L/W/H from reconstructed cloud with bootstrap CIs.
%
% Inputs:
%   recon -> reconstruction struct with world-frame point cloud
%
% Optional name-value pairs:
%   'CropX'            -> x crop bounds
%   'CropY'            -> y crop bounds
%   'CropZ'            -> z crop bounds
%   'LowQuantile'      -> lower quantile for robust bounds
%   'HighQuantile'     -> upper quantile for robust bounds
%   'NumBootstrap'     -> number of bootstrap resamples
%   'BootstrapCount'   -> number of points per bootstrap sample
%   'MakeFigure'       -> show plots
%   'Verbose'          -> print summary

    p = inputParser;
    addParameter(p, 'CropX', [-0.2 8.8], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'CropY', [-1.4 1.4], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'CropZ', [0.0 2.8], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'LowQuantile', 0.01, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x < 1);
    addParameter(p, 'HighQuantile', 0.99, @(x) isnumeric(x) && isscalar(x) && x > 0 && x <= 1);
    addParameter(p, 'NumBootstrap', 200, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'BootstrapCount', 15000, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'MakeFigure', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Verbose', true, @(x) islogical(x) || isnumeric(x));
    parse(p, varargin{:});

    cropX = double(p.Results.CropX(:))';
    cropY = double(p.Results.CropY(:))';
    cropZ = double(p.Results.CropZ(:))';
    qLo = double(p.Results.LowQuantile);
    qHi = double(p.Results.HighQuantile);
    nBoot = double(p.Results.NumBootstrap);
    nBootPts = double(p.Results.BootstrapCount);
    makeFigure = logical(p.Results.MakeFigure);
    verbose = logical(p.Results.Verbose);

    stats = analyze_pointcloud(recon, ...
        'CropX', cropX, ...
        'CropY', cropY, ...
        'CropZ', cropZ, ...
        'MakeFigure', false, ...
        'Verbose', false);

    pts = stats.points_cropped;

    if isempty(pts)
        error('No cropped points available for dimension inference.');
    end

    x = pts(:,1);
    y = pts(:,2);
    z = pts(:,3);

    xLo = quantile(x, qLo);
    xHi = quantile(x, qHi);
    yLo = quantile(y, qLo);
    yHi = quantile(y, qHi);
    zLo = quantile(z, qLo);
    zHi = quantile(z, qHi);

    L_est = xHi - xLo;
    W_est = yHi - yLo;
    H_est = zHi - zLo;

    nPts = size(pts,1);
    nBootPts = min(nBootPts, nPts);

    bootDims = zeros(nBoot, 3);

    for b = 1:nBoot
        idx = randi(nPts, [nBootPts, 1]);
        pb = pts(idx, :);

        xb = pb(:,1);
        yb = pb(:,2);
        zb = pb(:,3);

        bootDims(b,1) = quantile(xb, qHi) - quantile(xb, qLo);
        bootDims(b,2) = quantile(yb, qHi) - quantile(yb, qLo);
        bootDims(b,3) = quantile(zb, qHi) - quantile(zb, qLo);
    end

    L_ci = quantile(bootDims(:,1), [0.025 0.975]);
    W_ci = quantile(bootDims(:,2), [0.025 0.975]);
    H_ci = quantile(bootDims(:,3), [0.025 0.975]);

    gt = build_gt_mesh_jason_ship('MeshSpacing', 0.10);
    Vgt = gt.vertices;

    L_gt = max(Vgt(:,1)) - min(Vgt(:,1));
    W_gt = max(Vgt(:,2)) - min(Vgt(:,2));
    H_gt = max(Vgt(:,3)) - min(Vgt(:,3));

    dimResult = struct();
    dimResult.length_est = L_est;
    dimResult.width_est = W_est;
    dimResult.height_est = H_est;

    dimResult.length_ci_95 = L_ci;
    dimResult.width_ci_95 = W_ci;
    dimResult.height_ci_95 = H_ci;

    dimResult.length_gt = L_gt;
    dimResult.width_gt = W_gt;
    dimResult.height_gt = H_gt;

    dimResult.length_abs_error = abs(L_est - L_gt);
    dimResult.width_abs_error = abs(W_est - W_gt);
    dimResult.height_abs_error = abs(H_est - H_gt);

    dimResult.length_rel_error_pct = 100 * abs(L_est - L_gt) / max(L_gt, eps);
    dimResult.width_rel_error_pct = 100 * abs(W_est - W_gt) / max(W_gt, eps);
    dimResult.height_rel_error_pct = 100 * abs(H_est - H_gt) / max(H_gt, eps);

    dimResult.num_points_used = nPts;
    dimResult.quantile_low = qLo;
    dimResult.quantile_high = qHi;
    dimResult.bootstrap_dims = bootDims;
    dimResult.points_used = pts;

    if verbose
        fprintf('\n=== DIMENSION INFERENCE SUMMARY ===\n');
        fprintf('Points used: %d\n', nPts);

        fprintf('\nEstimated dimensions:\n');
        fprintf('  Length = %.6f  (95%% CI: [%.6f, %.6f])\n', L_est, L_ci(1), L_ci(2));
        fprintf('  Width  = %.6f  (95%% CI: [%.6f, %.6f])\n', W_est, W_ci(1), W_ci(2));
        fprintf('  Height = %.6f  (95%% CI: [%.6f, %.6f])\n', H_est, H_ci(1), H_ci(2));

        fprintf('\nGT dimensions:\n');
        fprintf('  Length = %.6f\n', L_gt);
        fprintf('  Width  = %.6f\n', W_gt);
        fprintf('  Height = %.6f\n', H_gt);

        fprintf('\nAbsolute errors:\n');
        fprintf('  Length = %.6f\n', dimResult.length_abs_error);
        fprintf('  Width  = %.6f\n', dimResult.width_abs_error);
        fprintf('  Height = %.6f\n', dimResult.height_abs_error);

        fprintf('===================================\n\n');
    end

    if makeFigure
        figure('Color', 'w', 'Name', 'Dimension Inference Bootstrap');
        tiledlayout(1,3,'Padding','compact','TileSpacing','compact');

        nexttile;
        histogram(bootDims(:,1), 30);
        xlabel('Length');
        ylabel('Count');
        title('Bootstrap Length');

        nexttile;
        histogram(bootDims(:,2), 30);
        xlabel('Width');
        ylabel('Count');
        title('Bootstrap Width');

        nexttile;
        histogram(bootDims(:,3), 30);
        xlabel('Height');
        ylabel('Count');
        title('Bootstrap Height');
    end
end