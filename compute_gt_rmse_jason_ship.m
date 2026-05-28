function metrics = compute_gt_rmse_jason_ship(recon, varargin)             % define function that computes proxy RMSE to Jason ship GT
%COMPUTE_GT_RMSE_JASON_SHIP Compute a proxy GT RMSE for Jason's ship.      % short description
%
% This script builds a GT proxy point set from the known Jason ship        % explain the idea
% geometry (hull, deck, bridge block, funnel, container stacks), then      % list main GT parts
% compares the cropped reconstructed cloud to that GT proxy using          % explain comparison direction
% nearest-neighbor distances.                                              % explain metric
%
% Inputs:                                                                  % document inputs
%   recon -> struct returned by reconstruct_pointcloud                     % expected reconstruction
%
% Optional name-value pairs:                                               % document options
%   'CropX', 'CropY', 'CropZ' -> crop box for evaluation                   % crop box
%   'GTSpacing'              -> spacing of GT sample points                % GT point spacing
%   'MaxEvalPoints'          -> cap on reconstruction points used          % speed control
%   'ChunkSize'              -> chunk size for nearest-neighbor search     % memory control
%   'MakeFigure'             -> show histogram/overlay                     % display flag
%   'Verbose'                -> print summary                              % print flag

    p = inputParser;                                                       % create parser
    addParameter(p, 'CropX', [-0.2 8.8], @(x) isnumeric(x) && numel(x)==2); % x crop bounds
    addParameter(p, 'CropY', [-1.4 1.4], @(x) isnumeric(x) && numel(x)==2); % y crop bounds
    addParameter(p, 'CropZ', [0.0 2.8], @(x) isnumeric(x) && numel(x)==2);  % z crop bounds
    addParameter(p, 'GTSpacing', 0.10, @(x) isnumeric(x) && isscalar(x) && x > 0); % GT sample spacing
    addParameter(p, 'MaxEvalPoints', 30000, @(x) isnumeric(x) && isscalar(x) && x > 0); % max recon points used
    addParameter(p, 'ChunkSize', 300, @(x) isnumeric(x) && isscalar(x) && x > 0); % chunk size for distance search
    addParameter(p, 'MakeFigure', true, @(x) islogical(x) || isnumeric(x)); % figure flag
    addParameter(p, 'Verbose', true, @(x) islogical(x) || isnumeric(x));    % print flag
    parse(p, varargin{:});                                                 % parse options

    cropX = double(p.Results.CropX(:))';                                   % store x crop
    cropY = double(p.Results.CropY(:))';                                   % store y crop
    cropZ = double(p.Results.CropZ(:))';                                   % store z crop
    gtSpacing = double(p.Results.GTSpacing);                               % store GT spacing
    maxEvalPoints = double(p.Results.MaxEvalPoints);                       % store max eval point cap
    chunkSize = double(p.Results.ChunkSize);                               % store chunk size
    makeFigure = logical(p.Results.MakeFigure);                            % convert figure flag
    verbose = logical(p.Results.Verbose);                                  % convert verbose flag

    stats = analyze_pointcloud(recon, ...                                  % crop and summarize the reconstruction first
        'CropX', cropX, ...                                                % pass x crop
        'CropY', cropY, ...                                                % pass y crop
        'CropZ', cropZ, ...                                                % pass z crop
        'MakeFigure', false, ...                                           % do not show extra figure here
        'Verbose', false);                                                 % do not print extra summary here

    ptsEval = stats.points_cropped;                                        % use cropped reconstructed points for evaluation
    colsEval = stats.colors_cropped;                                       % keep colors for optional overlay

    if isempty(ptsEval)                                                    % if no points survived crop
        error('No cropped points available for RMSE evaluation.');         % stop because RMSE cannot be computed
    end

    if size(ptsEval, 1) > maxEvalPoints                                    % if too many points for fast evaluation
        idxEval = round(linspace(1, size(ptsEval,1), maxEvalPoints));      % choose evenly spaced subset indices
        ptsEval = ptsEval(idxEval, :);                                     % downsample evaluation points
        colsEval = colsEval(idxEval, :);                                   % downsample colors too
    end

    gtPts = sample_jason_ship_gt_proxy(gtSpacing);                         % generate GT proxy points from known ship geometry

    d = nearest_distances_chunked(ptsEval, gtPts, chunkSize);              % compute nearest GT distance for each recon point

    rmse = sqrt(mean(d.^2));                                               % compute root-mean-square error
    mae = mean(d);                                                         % compute mean absolute error
    med = median(d);                                                       % compute median distance
    pct005 = mean(d <= 0.05) * 100;                                        % percentage within 5 cm
    pct010 = mean(d <= 0.10) * 100;                                        % percentage within 10 cm
    pct020 = mean(d <= 0.20) * 100;                                        % percentage within 20 cm

    metrics = struct();                                                    % create output struct
    metrics.rmse = rmse;                                                   % store RMSE
    metrics.mae = mae;                                                     % store MAE
    metrics.median_error = med;                                            % store median error
    metrics.pct_within_0_05 = pct005;                                      % store 5 cm coverage metric
    metrics.pct_within_0_10 = pct010;                                      % store 10 cm coverage metric
    metrics.pct_within_0_20 = pct020;                                      % store 20 cm coverage metric
    metrics.eval_point_count = size(ptsEval, 1);                           % store number of evaluated recon points
    metrics.gt_point_count = size(gtPts, 1);                               % store number of GT proxy points
    metrics.distances = d;                                                 % store per-point nearest distances
    metrics.points_eval = ptsEval;                                         % store evaluated recon points
    metrics.colors_eval = colsEval;                                        % store evaluated colors
    metrics.gt_points = gtPts;                                             % store GT proxy points used

    if verbose                                                             % if printing is enabled
        fprintf('\n=== JASON SHIP GT RMSE SUMMARY ===\n');                 % print header
        fprintf('Eval points used: %d\n', size(ptsEval,1));                % print number of recon points evaluated
        fprintf('GT proxy points:  %d\n', size(gtPts,1));                  % print number of GT points
        fprintf('GT spacing:       %.4f\n', gtSpacing);                    % print GT spacing
        fprintf('RMSE:             %.6f\n', rmse);                         % print RMSE
        fprintf('MAE:              %.6f\n', mae);                          % print MAE
        fprintf('Median error:     %.6f\n', med);                          % print median error
        fprintf('Pct <= 0.05 m:    %.2f\n', pct005);                      % print 5 cm coverage
        fprintf('Pct <= 0.10 m:    %.2f\n', pct010);                      % print 10 cm coverage
        fprintf('Pct <= 0.20 m:    %.2f\n', pct020);                      % print 20 cm coverage
        fprintf('===================================\n\n');               % print footer
    end

    if makeFigure                                                          % if figures are requested
        figure('Color', 'w');                                              % create histogram figure
        histogram(d, 60);                                                  % plot distance histogram
        ax1 = gca;                                                         % get current axes handle
        set(ax1, 'XColor', 'k', 'YColor', 'k');                            % make axis tick/axis colors black
        xlabel('Nearest GT distance (m)', 'Color', 'k');                   % x-axis label in black
        ylabel('Count', 'Color', 'k');                                     % y-axis label in black
        title(sprintf('GT Proxy Distance Histogram (RMSE = %.4f m)', rmse), 'Color', 'k'); % title in black

        figure('Color', 'w');                                              % create overlay figure
        nGT = min(5000, size(gtPts,1));                                    % cap GT points shown for speed
        idxGT = round(linspace(1, size(gtPts,1), nGT));                    % choose representative GT subset
        scatter3(gtPts(idxGT,1), gtPts(idxGT,2), gtPts(idxGT,3), 5, 'r', 'filled'); % plot GT subset in red
        hold on;                                                           % keep figure for recon overlay
        scatter3(ptsEval(:,1), ptsEval(:,2), ptsEval(:,3), 3, double(colsEval)/255, 'filled'); % plot recon points
        axis equal;                                                        % keep proper aspect ratio
        grid on;                                                           % show grid
        ax2 = gca;                                                         % get current axes handle
        set(ax2, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');            % make axis colors black
        xlabel('X', 'Color', 'k');                                         % x-axis label in black
        ylabel('Y', 'Color', 'k');                                         % y-axis label in black
        zlabel('Z', 'Color', 'k');                                         % z-axis label in black
        title('Reconstruction vs GT Proxy (red = GT sample)', 'Color', 'k'); % title in black
        view(3);                                                           % use 3D view
    end
end
function gtPts = sample_jason_ship_gt_proxy(spacing)                       % helper that samples a GT proxy point set
    ztop  = 0.5;                                                           % same base reference as Jason script
    zdeck = ztop + 0.52;                                                   % same deck height as Jason script
    zkeel = ztop - 0.10;                                                   % same keel height as Jason script
    depth = zdeck - zkeel;                                                 % same hull depth as Jason script

    nx = 45;                                                               % same hull profile resolution as Jason script
    n1 = round(nx * 0.20);                                                 % same first Bezier segment count
    n2 = nx - n1 - 1;                                                      % same second Bezier segment count

    p1 = [0   0    zdeck];                                                 % same first hull point
    p2 = [4.2 0.60 zdeck];                                                 % same second hull point
    p3 = [8.4 0    zdeck];                                                 % same third hull point

    alfa1 = 0.30;  alfa2 = 3.50;                                           % same Bezier parameters
    gamma1 = 3.50; gamma2 = 0.30;                                          % same Bezier parameters
    k1 = [4 3 0];  k2 = [1 0 0];  k3 = [-4 3 0];                          % same Bezier directions

    b1 = p1 + alfa1 * k1 / norm(k1);                                       % compute first Bezier control point
    c1 = p2 + gamma1 * (-k2);                                              % compute second control point
    b2 = p2 + alfa2 * k2;                                                  % compute third control point
    c2 = p3 + gamma2 * k3 / norm(k3);                                      % compute fourth control point

    rpos = [bezier(p1, b1, c1, p2, n1);                                    % sample first Bezier arc
            bezier(p2, b2, c2, p3, n2);                                    % sample second Bezier arc
            p3];                                                            % append endpoint

    x_ship = rpos(:,1);                                                    % hull x profile
    y_rail = rpos(:,2);                                                    % hull half-width profile

    gtPts = zeros(0,3);                                                    % initialize empty GT point array

    xq = 0:spacing:8.4;                                                    % x samples along ship length
    zq = zkeel:spacing:zdeck;                                              % z samples along hull side height

    for xi = 1:numel(xq)                                                   % loop over hull x samples
        yw = interp1(x_ship, y_rail, xq(xi), 'linear', 'extrap');          % get half-width at this x
        for zi = 1:numel(zq)                                               % loop over hull side z samples
            gtPts(end+1,:) = [xq(xi),  yw, zq(zi)]; %#ok<AGROW>            % add port hull side point
            gtPts(end+1,:) = [xq(xi), -yw, zq(zi)]; %#ok<AGROW>            % add starboard hull side point
        end
    end

    for xi = 1:numel(xq)                                                   % loop over deck x samples
        yw = interp1(x_ship, y_rail, xq(xi), 'linear', 'extrap');          % deck half-width at this x
        yq = -yw:spacing:yw;                                               % y samples across deck at this x
        if isempty(yq)                                                     % if spacing produced no samples
            yq = 0;                                                        % fall back to center sample
        end
        gtPts = [gtPts; [xq(xi)*ones(numel(yq),1), yq(:), zdeck*ones(numel(yq),1)]]; %#ok<AGROW> % add deck top samples
    end

    x0 = 1.34;  x1 = 1.84;  yw_s = 0.21;                                   % bridge block dimensions from Jason script
    z0 = zdeck; z1 = zdeck + 0.52;                                         % bridge block z limits from Jason script
    xs = x0:spacing:x1;                                                    % x samples along bridge block
    ys = -yw_s:spacing:yw_s;                                               % y samples across bridge block
    zs = z0:spacing:z1;                                                    % z samples along bridge block height

    [Ys, Zs] = meshgrid(ys, zs);                                           % grid for front/aft bridge faces
    gtPts = [gtPts; [x0*ones(numel(Ys),1), Ys(:), Zs(:)]]; %#ok<AGROW>    % add bridge front face
    gtPts = [gtPts; [x1*ones(numel(Ys),1), Ys(:), Zs(:)]]; %#ok<AGROW>    % add bridge aft face

    [Xs, Zs2] = meshgrid(xs, zs);                                          % grid for port/starboard bridge faces
    gtPts = [gtPts; [Xs(:),  yw_s*ones(numel(Xs),1), Zs2(:)]]; %#ok<AGROW> % add bridge port face
    gtPts = [gtPts; [Xs(:), -yw_s*ones(numel(Xs),1), Zs2(:)]]; %#ok<AGROW> % add bridge starboard face

    [Xs2, Ys2] = meshgrid(xs, ys);                                         % grid for bridge roof
    gtPts = [gtPts; [Xs2(:), Ys2(:), z1*ones(numel(Xs2),1)]]; %#ok<AGROW> % add bridge roof

    xc_s = (x0 + x1) / 2;                                                  % bridge center x used for funnel
    rf = 0.08;                                                             % funnel radius from Jason script
    zf = z1:spacing:(z1+0.38);                                             % funnel z samples
    tf = 0:spacing/rf:(2*pi);                                              % approximate angular samples around funnel
    if tf(end) < 2*pi                                                      % ensure full circle is sampled
        tf = [tf 2*pi];                                                    % append final angle
    end

    for zi = 1:numel(zf)                                                   % loop over funnel height
        xf = xc_s + rf*cos(tf);                                            % x ring coordinates
        yf = rf*sin(tf);                                                   % y ring coordinates
        gtPts = [gtPts; [xf(:), yf(:), zf(zi)*ones(numel(tf),1)]]; %#ok<AGROW> % add funnel ring
    end

    cL  = 0.58;                                                            % container length from Jason script
    cW  = 0.24;                                                            % container width from Jason script
    cH  = 0.27;                                                            % container height from Jason script
    gap = 0.015;                                                           % gap from Jason script
    x_start = x1 + 0.12;                                                   % container start x from Jason script
    x_end   = 8.00;                                                        % container end x from Jason script
    n_cols = 4;                                                            % number of container columns
    y_centers = (-(n_cols-1)/2 : 1 : (n_cols-1)/2) * (cW + gap);          % same column centers
    n_rows = floor((x_end - x_start) / (cL + gap));                        % same number of rows
    n_layers = 2;                                                          % same number of layers

    for rr = 1:n_rows                                                      % loop over container rows
        x0c = x_start + (rr-1)*(cL + gap);                                 % container x start
        x1c = x0c + cL;                                                    % container x end
        if x1c > x_end                                                     % if container exceeds deck end
            break;                                                         % stop building rows
        end
        for cc = 1:n_cols                                                  % loop over columns
            yc = y_centers(cc);                                            % current container center y
            y0c = yc - cW/2;                                               % container lower y
            y1c = yc + cW/2;                                               % container upper y
            for lyr = 1:n_layers                                           % loop over layers
                z0c = zdeck + (lyr-1)*(cH + gap);                          % container lower z
                z1c = z0c + cH;                                            % container upper z

                xface = x0c:spacing:x1c;                                   % x samples for this box
                yface = y0c:spacing:y1c;                                   % y samples for this box
                zface = z0c:spacing:z1c;                                   % z samples for this box

                [Yf, Zf] = meshgrid(yface, zface);                         % grid for front/back faces
                gtPts = [gtPts; [x0c*ones(numel(Yf),1), Yf(:), Zf(:)]]; %#ok<AGROW> % add front face
                gtPts = [gtPts; [x1c*ones(numel(Yf),1), Yf(:), Zf(:)]]; %#ok<AGROW> % add back face

                [Xf, Zf2] = meshgrid(xface, zface);                        % grid for side faces
                gtPts = [gtPts; [Xf(:), y0c*ones(numel(Xf),1), Zf2(:)]]; %#ok<AGROW> % add port face
                gtPts = [gtPts; [Xf(:), y1c*ones(numel(Xf),1), Zf2(:)]]; %#ok<AGROW> % add starboard face

                [Xtop, Ytop] = meshgrid(xface, yface);                     % grid for top face
                gtPts = [gtPts; [Xtop(:), Ytop(:), z1c*ones(numel(Xtop),1)]]; %#ok<AGROW> % add container top
            end
        end
    end

    gtPts = unique(round(gtPts, 6), 'rows');                               % remove duplicate GT points after rounding
end

function d = nearest_distances_chunked(P, Q, chunkSize)                    % helper that computes nearest GT distance for each point
    nP = size(P, 1);                                                       % number of query points
    q2 = sum(Q.^2, 2)';                                                    % squared norms of GT points
    d = zeros(nP, 1);                                                      % preallocate output distances

    for s = 1:chunkSize:nP                                                 % process query points in chunks
        e = min(s + chunkSize - 1, nP);                                    % end index for this chunk
        Pi = P(s:e, :);                                                    % current chunk of query points
        p2 = sum(Pi.^2, 2);                                                % squared norms of chunk points
        D2 = p2 + q2 - 2 * (Pi * Q');                                      % full squared-distance matrix for chunk
        D2 = max(D2, 0);                                                   % clamp tiny negative values from roundoff
        d(s:e) = sqrt(min(D2, [], 2));                                     % store nearest-neighbor distance for chunk
    end
end