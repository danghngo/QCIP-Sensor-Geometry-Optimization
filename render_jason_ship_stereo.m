function [renderData, rig, sceneInfo] = render_jason_ship_stereo(varargin)
%RENDER_JASON_SHIP_STEREO Run Jason's container-ship script, define a
%PARALLEL stereo rig, and render left/right images from the same MATLAB scene.
%
%   Renders a TRUE PARALLEL stereo pair:
%   - same orientation for left/right cameras
%   - different camera centers
%   - camera target for each eye is camPos + forwardDir * targetDistance
%     (NOT both eyes looking at the same scene center)

    % -----------------------------
    % Parse inputs
    % -----------------------------
    p = inputParser;
    addParameter(p, 'ShipScript', 'oil_tanker_generate_Tanker.m', @(x) ischar(x) || isstring(x));
    addParameter(p, 'AzimuthDeg', 25, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'ElevationDeg', 8, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Range', 12, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Baseline', 0.10, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'LookAt', [], @(x) isempty(x) || (isnumeric(x) && numel(x)==3));
    addParameter(p, 'ImageSize', [720 1280], @(x) isnumeric(x) && numel(x)==2);
    addParameter(p, 'HFOVDeg', 10, @(x) isnumeric(x) && isscalar(x) && x > 0 && x < 180);
    addParameter(p, 'MakeFigure', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Verbose', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'SaveImages', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ReturnHandles', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'CloseSceneFigure', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
    parse(p, varargin{:});

    shipScript  = char(p.Results.ShipScript);
    azDeg       = p.Results.AzimuthDeg;
    elDeg       = p.Results.ElevationDeg;
    rangeVal    = p.Results.Range;
    lookAtUser  = p.Results.LookAt;
    baseline    = p.Results.Baseline;
    imageSize   = p.Results.ImageSize;
    hfovDeg     = p.Results.HFOVDeg;
    makeFigure  = logical(p.Results.MakeFigure);
    verbose     = logical(p.Results.Verbose);
    saveImages  = logical(p.Results.SaveImages);
    returnHandles    = logical(p.Results.ReturnHandles);
    closeSceneFigure = p.Results.CloseSceneFigure;

    if isempty(closeSceneFigure)
        closeSceneFigure = ~makeFigure;
    else
        closeSceneFigure = logical(closeSceneFigure);
    end

    if closeSceneFigure && returnHandles
        warning('ReturnHandles ignored because CloseSceneFigure=true.');
        returnHandles = false;
    end

    % -----------------------------
    % Locate and run Jason script
    % -----------------------------
    thisDir = fileparts(mfilename('fullpath'));
    if isfile(fullfile(thisDir, shipScript))
        shipScriptPath = fullfile(thisDir, shipScript);
    elseif isfile(shipScript)
        shipScriptPath = shipScript;
    else
        error('Ship script not found: %s', shipScript);
    end

    run(shipScriptPath);

    fig = gcf;
    ax  = gca;

    if ~isgraphics(fig, 'figure')
        error('Current figure handle is invalid after running ship script.');
    end
    if ~isgraphics(ax, 'axes')
        error('Current axes handle is invalid after running ship script.');
    end

    sceneFigureCleanup = onCleanup(@() close_valid_figure(fig, closeSceneFigure)); %#ok<NASGU>

    % -----------------------------
    % Scene bounds from axes
    % -----------------------------
    xl = xlim(ax);
    yl = ylim(ax);
    zl = zlim(ax);

    sceneMin = [xl(1), yl(1), zl(1)];
    sceneMax = [xl(2), yl(2), zl(2)];
    sceneDim = sceneMax - sceneMin;
    sceneCtr = (sceneMin + sceneMax) / 2;

    if isempty(lookAtUser)
        lookAt = sceneCtr;
    else
        lookAt = reshape(lookAtUser, 1, 3);
    end

    % -----------------------------
    % Build PARALLEL stereo rig
    % -----------------------------
    az = deg2rad(azDeg);
    el = deg2rad(elDeg);

    rHat = [cos(el)*cos(az), ...
            cos(el)*sin(az), ...
            sin(el)];
    rHat = normalize_row(rHat);

    C_sat = lookAt + rangeVal * rHat;

    baselineHat = [-sin(az), cos(az), 0];
    if norm(baselineHat) < 1e-12
        baselineHat = [0 1 0];
    end
    baselineHat = normalize_row(baselineHat);

    halfBaseline = baseline / 2;

    % Shared camera orientation
    z_cam = normalize_row(lookAt - C_sat);
    x_temp = baselineHat - dot(baselineHat, z_cam) * z_cam;
    if norm(x_temp) < 1e-12
        x_temp = cross([0 0 1], z_cam);
        if norm(x_temp) < 1e-12
            x_temp = cross([0 1 0], z_cam);
        end
    end
    x_cam = normalize_row(x_temp);
    y_down = normalize_row(cross(z_cam, x_cam));

    R_wc = [x_cam; y_down; z_cam];

    C_left  = C_sat - halfBaseline * baselineHat;
    C_right = C_sat + halfBaseline * baselineHat;

    % MATLAB rendering needs image-UP for camup
    camUp = -y_down;

    % Use forward direction to create a true parallel target
    targetDistance = max(10 * norm(sceneDim), rangeVal);

    % -----------------------------
    % Intrinsics
    % -----------------------------
    imgH = imageSize(1);
    imgW = imageSize(2);

    fx = (imgW / 2) / tand(hfovDeg / 2);
    fy = fx;
    cx = (imgW + 1) / 2;
    cy = (imgH + 1) / 2;

    K = [fx  0  cx;
          0  fy cy;
          0  0  1];

    % -----------------------------
    % Normalize scene appearance
    % -----------------------------
    set(fig, 'Color', 'w', 'Renderer', 'opengl');
    set(ax, 'Color', 'w');
    axis(ax, 'vis3d');
    drawnow;

    delete(findall(fig, 'Type', 'light'));

    L = max(sceneDim);
    axes(ax);
    lighting gouraud;
    material dull;
    light('Parent', ax, 'Position', [ sceneCtr(1)+1.7*L, sceneCtr(2)-1.2*L, sceneCtr(3)+2.2*L], 'Style', 'infinite');
    light('Parent', ax, 'Position', [ sceneCtr(1)-1.0*L, sceneCtr(2)+0.9*L, sceneCtr(3)+1.5*L], 'Style', 'infinite');

    % -----------------------------
    % Convert horizontal -> vertical FOV for MATLAB camera
    % -----------------------------
    vfovDeg = 2 * atan( tand(hfovDeg/2) * (imgH / imgW) );
    vfovDeg = rad2deg(vfovDeg);

    % -----------------------------
    % Render left and right
    % -----------------------------
    leftRGB = capture_camera_view(fig, ax, C_left, z_cam, camUp, targetDistance, vfovDeg, imgW, imgH);
    rightRGB = capture_camera_view(fig, ax, C_right, z_cam, camUp, targetDistance, vfovDeg, imgW, imgH);

    leftGray  = rgb2gray(leftRGB);
    rightGray = rgb2gray(rightRGB);

    % -----------------------------
    % Outputs
    % -----------------------------
    renderData = struct();
    renderData.left_rgb   = leftRGB;
    renderData.right_rgb  = rightRGB;
    renderData.left_gray  = leftGray;
    renderData.right_gray = rightGray;
    renderData.image_size = [imgH imgW];
    renderData.vfov_deg   = vfovDeg;

    rig = struct();
    rig.look_at         = lookAt;
    rig.azimuth_deg     = azDeg;
    rig.elevation_deg   = elDeg;
    rig.range           = rangeVal;
    rig.baseline        = baseline;
    rig.half_baseline   = halfBaseline;
    rig.r_hat           = rHat;
    rig.baseline_hat    = baselineHat;
    rig.C_sat           = C_sat;
    rig.C_left          = C_left;
    rig.C_right         = C_right;
    rig.R_left_wc       = R_wc;
    rig.R_right_wc      = R_wc;
    rig.x_cam           = x_cam;
    rig.y_down          = y_down;
    rig.z_cam           = z_cam;
    rig.y_left          = camUp;
    rig.y_right         = camUp;
    rig.image_size      = [imgH imgW];
    rig.hfov_deg        = hfovDeg;
    rig.K               = K;
    rig.fx              = fx;
    rig.fy              = fy;
    rig.cx              = cx;
    rig.cy              = cy;

    sceneInfo = struct();

    if returnHandles
        sceneInfo.figure = fig;
        sceneInfo.axes   = ax;
    else
        sceneInfo.figure = [];
        sceneInfo.axes   = [];
    end

    sceneInfo.scene_center     = sceneCtr;
    sceneInfo.scene_min        = sceneMin;
    sceneInfo.scene_max        = sceneMax;
    sceneInfo.scene_dimensions = sceneDim;
    sceneInfo.ship_script      = shipScriptPath;
    sceneInfo.closed_on_exit   = closeSceneFigure;

    if verbose
        fprintf('\n=== JASON SHIP STEREO RENDER SUMMARY ===\n');
        fprintf('Ship script: %s\n', shipScriptPath);
        fprintf('Scene center: [%.6f, %.6f, %.6f]\n', sceneCtr(1), sceneCtr(2), sceneCtr(3));
        fprintf('Scene dims:   [%.6f, %.6f, %.6f]\n', sceneDim(1), sceneDim(2), sceneDim(3));
        fprintf('Azimuth (deg):   %.3f\n', azDeg);
        fprintf('Elevation (deg): %.3f\n', elDeg);
        fprintf('Range:           %.6f\n', rangeVal);
        fprintf('Baseline:        %.6f\n', baseline);
        fprintf('Image size:      [%d, %d]\n', imgH, imgW);
        fprintf('HFOV (deg):      %.3f\n', hfovDeg);
        fprintf('fx = %.6f, fy = %.6f\n', fx, fy);
        fprintf('cx = %.6f, cy = %.6f\n', cx, cy);
        fprintf('========================================\n\n');
    end

    if makeFigure
        figure('Color', 'w', 'Name', 'Jason Ship Stereo Pair');
        tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

        nexttile;
        imshow(leftRGB);
        title('Left RGB');

        nexttile;
        imshow(rightRGB);
        title('Right RGB');

        nexttile;
        imshow(leftGray);
        title('Left Grayscale');

        nexttile;
        imshow(rightGray);
        title('Right Grayscale');
    end

    if saveImages
        outDir = fullfile(thisDir, '..', 'images');
        if ~exist(outDir, 'dir')
            mkdir(outDir);
        end
        imwrite(leftRGB,   fullfile(outDir, 'jason_ship_left_rgb.png'));
        imwrite(rightRGB,  fullfile(outDir, 'jason_ship_right_rgb.png'));
        imwrite(leftGray,  fullfile(outDir, 'jason_ship_left_gray.png'));
        imwrite(rightGray, fullfile(outDir, 'jason_ship_right_gray.png'));
    end
end

% ==========================================================
% Helper: capture one camera view from the existing scene
% ==========================================================
function rgb = capture_camera_view(fig, ax, camPos, camDir, camUp, targetDistance, vfovDeg, imgW, imgH)
    set(fig, 'Units', 'pixels', 'Position', [100 100 imgW imgH]);
    set(ax, 'Units', 'normalized');
    set(ax, 'Position', [0 0 1 1]);
    set(ax, 'LooseInset', [0 0 0 0]);
    title(ax, '');

    camTarget = camPos + camDir * targetDistance;

    campos(ax, camPos);
    camtarget(ax, camTarget);
    camup(ax, camUp);
    camva(ax, vfovDeg);
    camproj(ax, 'perspective');

    drawnow;
    frame = getframe(fig);
    rgb = frame2im(frame);

    if size(rgb,1) ~= imgH || size(rgb,2) ~= imgW
        rgb = imresize(rgb, [imgH imgW]);
    end
end

% ==========================================================
% Helper
% ==========================================================
function v = normalize_row(v)
    n = norm(v);
    if n < 1e-12
        error('Cannot normalize near-zero vector.');
    end
    v = v / n;
end

function close_valid_figure(fig, doClose)
    if ~doClose
        return;
    end
    if ~isempty(fig) && isgraphics(fig, 'figure')
        delete(fig);
        drawnow;
    end
end