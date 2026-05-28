function metrics = compute_gt_metrics_jason_ship(recon, varargin)
%COMPUTE_GT_METRICS_JASON_SHIP Compute centroid, mesh-surface, symmetric,
%coverage, and density metrics for the tanker reconstruction.
%
% Metrics returned:
%   - proxy_one_sided_rmse
%   - proxy_symmetric_rmse
%   - centroid_one_sided_rmse
%   - centroid_symmetric_rmse
%   - mesh_surface_one_sided_rmse
%   - GT triangle coverage fractions at 0.05 / 0.10 / 0.20 m
%   - GT area coverage fractions at 0.05 / 0.10 / 0.20 m
%   - cropped-point density over total GT area and covered GT area
%
% Search-safe defaults:
%   MakeFigure = false
%   Verbose = false
%   ReturnHeavyFields = false

    p = inputParser;
    addParameter(p, 'CropX', [-0.2 8.8], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'CropY', [-1.4 1.4], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'CropZ', [0.0 2.8], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'GTMeshSpacing', 0.10, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'MaxEvalPoints', 30000, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'ChunkSizeNN', 300, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'ChunkSizeMesh', 120, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'TriangleCandidates', 12, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'MakeFigure', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Verbose', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ReturnHeavyFields', false, @(x) islogical(x) || isnumeric(x));
    parse(p, varargin{:});

    cropX = double(p.Results.CropX(:))';
    cropY = double(p.Results.CropY(:))';
    cropZ = double(p.Results.CropZ(:))';
    gtMeshSpacing = double(p.Results.GTMeshSpacing);
    maxEvalPoints = double(p.Results.MaxEvalPoints);
    chunkSizeNN = double(p.Results.ChunkSizeNN);
    chunkSizeMesh = double(p.Results.ChunkSizeMesh);
    triCandidates = double(p.Results.TriangleCandidates);
    makeFigure = logical(p.Results.MakeFigure);
    verbose = logical(p.Results.Verbose);
    returnHeavyFields = logical(p.Results.ReturnHeavyFields);

    stats = analyze_pointcloud(recon, ...
        'CropX', cropX, ...
        'CropY', cropY, ...
        'CropZ', cropZ, ...
        'MakeFigure', false, ...
        'Verbose', false);

    ptsAll = stats.points_cropped;
    colsAll = stats.colors_cropped;
    rawPointCount = stats.raw_count;
    croppedPointCount = stats.cropped_count;
    keepFraction = stats.keep_fraction;

    if isempty(ptsAll)
        error('No cropped reconstructed points available for GT metric computation.');
    end

    ptsEval = ptsAll;
    colsEval = colsAll;

    if size(ptsEval, 1) > maxEvalPoints
        idx = round(linspace(1, size(ptsEval,1), maxEvalPoints));
        ptsEval = ptsEval(idx, :);
        colsEval = colsEval(idx, :);
    end

    gt = build_gt_mesh_jason_ship('MeshSpacing', gtMeshSpacing);
    gtProxy = gt.proxy_points;
    gtCentroids = gt.face_centroids;
    triAreas = triangle_areas(gt.vertices, gt.faces);
    totalGtArea = sum(triAreas);

    % ------------------------------------------------------------
    % Proxy point-set metrics
    % ------------------------------------------------------------
    d_proxy_rg = nearest_distances_chunked(ptsEval, gtProxy, chunkSizeNN);
    d_proxy_gr = nearest_distances_chunked(gtProxy, ptsEval, chunkSizeNN);

    % ------------------------------------------------------------
    % Triangle-centroid metrics
    % ------------------------------------------------------------
    d_cent_rg = nearest_distances_chunked(ptsEval, gtCentroids, chunkSizeNN);
    d_cent_gr = nearest_distances_chunked(gtCentroids, ptsEval, chunkSizeNN);

    % ------------------------------------------------------------
    % True mesh-surface metric (one-sided)
    % ------------------------------------------------------------
    d_mesh_rg = point_to_mesh_distances_chunked(ptsEval, gt.vertices, gt.faces, chunkSizeMesh, triCandidates);

    % ------------------------------------------------------------
    % Coverage metrics based on GT centroids -> reconstruction
    % ------------------------------------------------------------
    thresh = [0.05 0.10 0.20];

    covTriCount = zeros(1, numel(thresh));
    covTriFrac = zeros(1, numel(thresh));
    covArea = zeros(1, numel(thresh));
    covAreaFrac = zeros(1, numel(thresh));

    for k = 1:numel(thresh)
        mask = d_cent_gr <= thresh(k);
        covTriCount(k) = nnz(mask);
        covTriFrac(k) = mean(mask);
        covArea(k) = sum(triAreas(mask));
        covAreaFrac(k) = covArea(k) / totalGtArea;
    end

    % ------------------------------------------------------------
    % Density metrics
    % ------------------------------------------------------------
    ptsPerTotalGtArea = croppedPointCount / totalGtArea;
    ptsPerCoveredArea005 = croppedPointCount / max(covArea(1), eps);
    ptsPerCoveredArea010 = croppedPointCount / max(covArea(2), eps);
    ptsPerCoveredArea020 = croppedPointCount / max(covArea(3), eps);

    metrics = struct();

    % Basic counts
    metrics.raw_point_count = rawPointCount;
    metrics.cropped_point_count = croppedPointCount;
    metrics.keep_fraction = keepFraction;
    metrics.gt_vertex_count = size(gt.vertices,1);
    metrics.gt_triangle_count = size(gt.faces,1);
    metrics.eval_point_count = size(ptsEval,1);
    metrics.total_gt_area = totalGtArea;

    % Accuracy metrics
    metrics.proxy_one_sided_rmse = sqrt(mean(d_proxy_rg.^2, 'omitnan'));
    metrics.proxy_symmetric_rmse = sqrt(mean([d_proxy_rg.^2; d_proxy_gr.^2], 'omitnan'));
    metrics.proxy_one_sided_mae = mean(d_proxy_rg, 'omitnan');
    metrics.proxy_symmetric_mae = mean([d_proxy_rg; d_proxy_gr], 'omitnan');

    metrics.centroid_one_sided_rmse = sqrt(mean(d_cent_rg.^2, 'omitnan'));
    metrics.centroid_symmetric_rmse = sqrt(mean([d_cent_rg.^2; d_cent_gr.^2], 'omitnan'));
    metrics.centroid_one_sided_mae = mean(d_cent_rg, 'omitnan');
    metrics.centroid_symmetric_mae = mean([d_cent_rg; d_cent_gr], 'omitnan');

    metrics.mesh_surface_one_sided_rmse = sqrt(mean(d_mesh_rg.^2, 'omitnan'));
    metrics.mesh_surface_one_sided_mae = mean(d_mesh_rg, 'omitnan');

    % Coverage metrics
    metrics.covered_gt_triangle_count_005 = covTriCount(1);
    metrics.covered_gt_triangle_count_010 = covTriCount(2);
    metrics.covered_gt_triangle_count_020 = covTriCount(3);

    metrics.covered_gt_triangle_fraction_005 = covTriFrac(1);
    metrics.covered_gt_triangle_fraction_010 = covTriFrac(2);
    metrics.covered_gt_triangle_fraction_020 = covTriFrac(3);

    metrics.covered_gt_area_005 = covArea(1);
    metrics.covered_gt_area_010 = covArea(2);
    metrics.covered_gt_area_020 = covArea(3);

    metrics.covered_gt_area_fraction_005 = covAreaFrac(1);
    metrics.covered_gt_area_fraction_010 = covAreaFrac(2);
    metrics.covered_gt_area_fraction_020 = covAreaFrac(3);

    % Density metrics
    metrics.cropped_points_per_total_gt_area = ptsPerTotalGtArea;
    metrics.cropped_points_per_covered_gt_area_005 = ptsPerCoveredArea005;
    metrics.cropped_points_per_covered_gt_area_010 = ptsPerCoveredArea010;
    metrics.cropped_points_per_covered_gt_area_020 = ptsPerCoveredArea020;

    % Optional heavy fields only when explicitly requested
    if returnHeavyFields
        metrics.proxy_recon_to_gt_distances = d_proxy_rg;
        metrics.proxy_gt_to_recon_distances = d_proxy_gr;
        metrics.centroid_recon_to_gt_distances = d_cent_rg;
        metrics.centroid_gt_to_recon_distances = d_cent_gr;
        metrics.mesh_surface_recon_to_gt_distances = d_mesh_rg;

        metrics.gt_mesh = gt;
        metrics.points_eval = ptsEval;
        metrics.colors_eval = colsEval;
    end

    if verbose
        fprintf('\n=== JASON SHIP UPGRADED GT METRICS SUMMARY ===\n');
        fprintf('Raw recon points:         %d\n', rawPointCount);
        fprintf('Cropped recon points:     %d\n', croppedPointCount);
        fprintf('Keep fraction:            %.6f\n', keepFraction);
        fprintf('Eval recon points:        %d\n', size(ptsEval,1));
        fprintf('GT mesh vertices:         %d\n', size(gt.vertices,1));
        fprintf('GT mesh triangles:        %d\n', size(gt.faces,1));
        fprintf('Total GT area:            %.6f\n', totalGtArea);

        fprintf('\nAccuracy metrics:\n');
        fprintf('  Proxy one-sided RMSE:   %.6f\n', metrics.proxy_one_sided_rmse);
        fprintf('  Proxy symmetric RMSE:   %.6f\n', metrics.proxy_symmetric_rmse);
        fprintf('  Centroid one-sided RMSE:%.6f\n', metrics.centroid_one_sided_rmse);
        fprintf('  Centroid symmetric RMSE:%.6f\n', metrics.centroid_symmetric_rmse);
        fprintf('  Mesh-surface RMSE:      %.6f\n', metrics.mesh_surface_one_sided_rmse);

        fprintf('\nCoverage metrics:\n');
        fprintf('  GT tri frac <= 0.05 m:  %.6f\n', metrics.covered_gt_triangle_fraction_005);
        fprintf('  GT tri frac <= 0.10 m:  %.6f\n', metrics.covered_gt_triangle_fraction_010);
        fprintf('  GT tri frac <= 0.20 m:  %.6f\n', metrics.covered_gt_triangle_fraction_020);
        fprintf('  GT area frac <= 0.05 m: %.6f\n', metrics.covered_gt_area_fraction_005);
        fprintf('  GT area frac <= 0.10 m: %.6f\n', metrics.covered_gt_area_fraction_010);
        fprintf('  GT area frac <= 0.20 m: %.6f\n', metrics.covered_gt_area_fraction_020);

        fprintf('\nDensity metrics:\n');
        fprintf('  Cropped pts / total GT area:        %.6f\n', metrics.cropped_points_per_total_gt_area);
        fprintf('  Cropped pts / covered GT area 0.05: %.6f\n', metrics.cropped_points_per_covered_gt_area_005);
        fprintf('  Cropped pts / covered GT area 0.10: %.6f\n', metrics.cropped_points_per_covered_gt_area_010);
        fprintf('  Cropped pts / covered GT area 0.20: %.6f\n', metrics.cropped_points_per_covered_gt_area_020);

        fprintf('================================================\n\n');
    end

    if makeFigure
        fig1 = figure('Color', 'w', 'Name', 'GT Metric Histograms');
        tiledlayout(1,3,'Padding','compact','TileSpacing','compact');

        nexttile;
        histogram(d_proxy_rg, 50);
        ax1 = gca; set(ax1, 'XColor', 'k', 'YColor', 'k');
        xlabel('Distance (m)', 'Color', 'k');
        ylabel('Count', 'Color', 'k');
        title(sprintf('Proxy NN RMSE = %.4f', metrics.proxy_one_sided_rmse), 'Color', 'k');

        nexttile;
        histogram(d_cent_rg, 50);
        ax2 = gca; set(ax2, 'XColor', 'k', 'YColor', 'k');
        xlabel('Distance (m)', 'Color', 'k');
        ylabel('Count', 'Color', 'k');
        title(sprintf('Centroid RMSE = %.4f', metrics.centroid_one_sided_rmse), 'Color', 'k');

        nexttile;
        histogram(d_mesh_rg, 50);
        ax3 = gca; set(ax3, 'XColor', 'k', 'YColor', 'k');
        xlabel('Distance (m)', 'Color', 'k');
        ylabel('Count', 'Color', 'k');
        title(sprintf('Mesh Surface RMSE = %.4f', metrics.mesh_surface_one_sided_rmse), 'Color', 'k');

        fig2 = figure('Color', 'w', 'Name', 'Recon vs GT Overlay');
        nGT = min(6000, size(gtCentroids,1));
        idxGT = round(linspace(1, size(gtCentroids,1), nGT));
        scatter3(gtCentroids(idxGT,1), gtCentroids(idxGT,2), gtCentroids(idxGT,3), 6, 'r', 'filled');
        hold on;
        scatter3(ptsEval(:,1), ptsEval(:,2), ptsEval(:,3), 3, double(colsEval)/255, 'filled');
        axis equal;
        grid on;
        ax4 = gca;
        set(ax4, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
        xlabel('X', 'Color', 'k');
        ylabel('Y', 'Color', 'k');
        zlabel('Z', 'Color', 'k');
        title('Recon vs GT Triangle Centroids (red = GT)', 'Color', 'k');
        view(3);

        fig3 = figure('Color', 'w', 'Name', 'GT Coverage');
        tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

        nexttile;
        bar(thresh, 100 * covTriFrac);
        ax5 = gca; set(ax5, 'XColor', 'k', 'YColor', 'k');
        xlabel('Threshold (m)', 'Color', 'k');
        ylabel('Covered GT triangles (%)', 'Color', 'k');
        title('GT Triangle Coverage', 'Color', 'k');

        nexttile;
        bar(thresh, 100 * covAreaFrac);
        ax6 = gca; set(ax6, 'XColor', 'k', 'YColor', 'k');
        xlabel('Threshold (m)', 'Color', 'k');
        ylabel('Covered GT area (%)', 'Color', 'k');
        title('GT Area Coverage', 'Color', 'k');

        drawnow;
        %#ok<NASGU>
        fig1; fig2; fig3;
    end
end

function d = nearest_distances_chunked(P, Q, chunkSize)
    nP = size(P,1);
    q2 = sum(Q.^2, 2)';
    d = zeros(nP,1);

    for s = 1:chunkSize:nP
        e = min(s + chunkSize - 1, nP);
        Pi = P(s:e,:);
        p2 = sum(Pi.^2, 2);
        D2 = p2 + q2 - 2*(Pi*Q');
        D2 = max(D2, 0);
        d(s:e) = sqrt(min(D2, [], 2));
    end
end

function d = point_to_mesh_distances_chunked(P, V, F, chunkSize, kCand)
    triC = (V(F(:,1),:) + V(F(:,2),:) + V(F(:,3),:)) / 3;
    triC2 = sum(triC.^2, 2)';
    nP = size(P,1);
    d = zeros(nP,1);

    for s = 1:chunkSize:nP
        e = min(s + chunkSize - 1, nP);
        Pi = P(s:e,:);
        p2 = sum(Pi.^2, 2);
        D2c = p2 + triC2 - 2*(Pi*triC');
        D2c = max(D2c, 0);

        [~, idxSort] = sort(D2c, 2, 'ascend');
        idxCand = idxSort(:, 1:min(kCand, size(idxSort,2)));

        for ii = 1:size(Pi,1)
            p = Pi(ii,:);
            triIdx = idxCand(ii,:);
            best = inf;

            for kk = 1:numel(triIdx)
                f = triIdx(kk);
                a = V(F(f,1),:);
                b = V(F(f,2),:);
                c = V(F(f,3),:);
                dsq = point_triangle_distance_sq(p, a, b, c);

                if isfinite(dsq) && dsq < best
                    best = dsq;
                end
            end

            if ~isfinite(best)
                candD2 = D2c(ii, triIdx);
                candD2 = candD2(isfinite(candD2));
                if isempty(candD2)
                    allD2 = D2c(ii, :);
                    allD2 = allD2(isfinite(allD2));
                    if isempty(allD2)
                        best = 0;
                    else
                        best = min(allD2);
                    end
                else
                    best = min(candD2);
                end
            end

            d(s + ii - 1) = sqrt(best);
        end
    end
end

function dsq = point_triangle_distance_sq(p, a, b, c)
    ab = b - a;
    ac = c - a;
    ap = p - a;

    d1 = dot(ab, ap);
    d2 = dot(ac, ap);
    if d1 <= 0 && d2 <= 0
        dsq = sum((p - a).^2);
        return;
    end

    bp = p - b;
    d3 = dot(ab, bp);
    d4 = dot(ac, bp);
    if d3 >= 0 && d4 <= d3
        dsq = sum((p - b).^2);
        return;
    end

    vc = d1*d4 - d3*d2;
    if vc <= 0 && d1 >= 0 && d3 <= 0
        v = d1 / (d1 - d3);
        proj = a + v*ab;
        dsq = sum((p - proj).^2);
        return;
    end

    cp = p - c;
    d5 = dot(ab, cp);
    d6 = dot(ac, cp);
    if d6 >= 0 && d5 <= d6
        dsq = sum((p - c).^2);
        return;
    end

    vb = d5*d2 - d1*d6;
    if vb <= 0 && d2 >= 0 && d6 <= 0
        w = d2 / (d2 - d6);
        proj = a + w*ac;
        dsq = sum((p - proj).^2);
        return;
    end

    va = d3*d6 - d5*d4;
    if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        proj = b + w*(c - b);
        dsq = sum((p - proj).^2);
        return;
    end

    denom = 1 / (va + vb + vc);
    v = vb * denom;
    w = vc * denom;
    proj = a + ab*v + ac*w;
    dsq = sum((p - proj).^2);
end

function A = triangle_areas(V, F)
    e1 = V(F(:,2),:) - V(F(:,1),:);
    e2 = V(F(:,3),:) - V(F(:,1),:);
    cp = cross(e1, e2, 2);
    A = 0.5 * sqrt(sum(cp.^2, 2));
end