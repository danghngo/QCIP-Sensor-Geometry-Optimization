function recon = reconstruct_pointcloud(renderData, rig, varargin)          % define function returning reconstruction struct
%RECONSTRUCT_POINTCLOUD Reconstruct 3D from a PARALLEL stereo pair.         % short description
%
% This version is specifically for the current synthetic pipeline where     % explain current assumption
% left/right images are already rendered from a PARALLEL stereo rig.        % images already share parallel geometry
% Therefore, it does NOT rectify again.                                     % no extra rectification step
%
% Usage:                                                                    % usage examples
%   recon = reconstruct_pointcloud(renderData, rig);                        % simplest call
%
%   recon = reconstruct_pointcloud(renderData, rig, ...                     % optional arguments example
%       'DisparityRange', [0 96], ...                                       % disparity search interval
%       'MakeFigure', true, ...                                             % show figures
%       'Verbose', true, ...                                                % print summary
%       'SavePointCloud', false);                                           % do not save point cloud
%
% Required renderData fields:                                               % expected image inputs
%   renderData.left_gray                                                    % left grayscale image
%   renderData.right_gray                                                   % right grayscale image
%
% Required rig fields:                                                      % expected rig inputs
%   rig.fx, rig.fy, rig.cx, rig.cy                                          % intrinsics
%   rig.R_left_wc                                                           % left world-to-camera rotation
%   rig.C_left, rig.C_right                                                 % left/right camera centers
%
% Output:                                                                   % returned fields
%   recon.left_gray_input                                                   % original left image
%   recon.right_gray_input                                                  % original right image
%   recon.left_gray_preprocessed                                            % preprocessed left image
%   recon.right_gray_preprocessed                                           % preprocessed right image
%   recon.disparity_map                                                     % estimated disparity map
%   recon.valid_mask                                                        % valid disparity mask
%   recon.points_left_camera                                                % 3D points in left camera frame
%   recon.points_world_approx                                               % 3D points mapped approximately to world
%   recon.ptCloud                                                           % MATLAB point cloud object

    % -----------------------------                                         % section divider
    % Checks                                                                % verify needed functions and fields exist
    % -----------------------------                                         % section divider
    requiredFns = {'disparitySGM', 'pointCloud'};                           % toolbox functions needed
    for i = 1:numel(requiredFns)                                            % loop through required functions
        if exist(requiredFns{i}, 'file') ~= 2                               % if a function is missing
            error('Required toolbox function not found: %s', requiredFns{i}); % stop with error
        end
    end

    requiredRenderFields = {'left_gray','right_gray'};                      % required image fields
    requiredRigFields = {'fx','fy','cx','cy','R_left_wc','C_left','C_right'}; % required rig fields

    for i = 1:numel(requiredRenderFields)                                   % check image fields
        if ~isfield(renderData, requiredRenderFields{i})                    % if a field is missing
            error('renderData missing required field: %s', requiredRenderFields{i}); % stop with error
        end
    end

    for i = 1:numel(requiredRigFields)                                      % check rig fields
        if ~isfield(rig, requiredRigFields{i})                              % if a rig field is missing
            error('rig missing required field: %s', requiredRigFields{i});  % stop with error
        end
    end

    % -----------------------------                                         % section divider
    % Parse options                                                         % read optional user settings
    % -----------------------------                                         % section divider
    p = inputParser;                                                        % create input parser
    addParameter(p, 'DisparityRange', [0 96], @(x) isnumeric(x) && numel(x)==2); % disparity search interval
    addParameter(p, 'MakeFigure', true, @(x) islogical(x) || isnumeric(x)); % show figures or not
    addParameter(p, 'Verbose', true, @(x) islogical(x) || isnumeric(x));   % print summary or not
    addParameter(p, 'SavePointCloud', false, @(x) islogical(x) || isnumeric(x)); % save point cloud or not
    addParameter(p, 'PointCloudFile', 'synthetic_reconstruction.ply', @(x) ischar(x) || isstring(x)); % output file name
    addParameter(p, 'MaxDepth', 1000, @(x) isnumeric(x) && isscalar(x) && x > 0); % max accepted depth
    addParameter(p, 'UseAdaptiveHistEq', true, @(x) islogical(x) || isnumeric(x)); % contrast enhancement toggle
    addParameter(p, 'BackgroundThreshold', 250, @(x) isnumeric(x) && isscalar(x)); % threshold for bright background
    addParameter(p, 'MinValidDisparity', 0.5, @(x) isnumeric(x) && isscalar(x) && x >= 0); % minimum disparity to trust
    parse(p, varargin{:});                                                  % parse all optional arguments

    dispRange           = double(p.Results.DisparityRange(:))';             % store disparity range as row vector
    makeFigure          = logical(p.Results.MakeFigure);                    % convert figure flag to logical
    verbose             = logical(p.Results.Verbose);                       % convert verbose flag to logical
    savePointCloud      = logical(p.Results.SavePointCloud);                % convert save flag to logical
    pointCloudFile      = char(p.Results.PointCloudFile);                   % store output file name
    maxDepth            = double(p.Results.MaxDepth);                       % store max depth
    useAdaptiveHistEq   = logical(p.Results.UseAdaptiveHistEq);             % store preprocessing choice
    backgroundThreshold = double(p.Results.BackgroundThreshold);            % store background threshold
    minValidDisparity   = double(p.Results.MinValidDisparity);              % store minimum disparity

    if dispRange(2) <= dispRange(1)                                         % if disparity range is invalid
        error('DisparityRange must satisfy max > min.');                    % stop with error
    end

    % -----------------------------                                         % section divider
    % Input images                                                          % get stereo images from input struct
    % -----------------------------                                         % section divider
    I1 = renderData.left_gray;                                              % load left grayscale image
    I2 = renderData.right_gray;                                             % load right grayscale image

    if ndims(I1) ~= 2 || ndims(I2) ~= 2                                     % ensure both are grayscale images
        error('Expected grayscale images in renderData.left_gray/right_gray.'); % stop if not grayscale
    end

    if ~isa(I1, 'uint8')                                                    % if left image is not uint8
        I1 = im2uint8(I1);                                                  % convert it to uint8
    end
    if ~isa(I2, 'uint8')                                                    % if right image is not uint8
        I2 = im2uint8(I2);                                                  % convert it to uint8
    end

    I1_input = I1;                                                          % preserve original left image
    I2_input = I2;                                                          % preserve original right image

    % -----------------------------                                         % section divider
    % Preprocess for better matching                                        % improve contrast before disparity
    % -----------------------------                                         % section divider
    if useAdaptiveHistEq                                                    % if adaptive histogram equalization is enabled
        I1 = adapthisteq(I1);                                               % enhance left image contrast
        I2 = adapthisteq(I2);                                               % enhance right image contrast
    end

    [H, W] = size(I1);                                                      % read image height and width

    % -----------------------------                                         % section divider
    % Compute disparity directly on RAW stereo pair                         % run stereo matcher
    % -----------------------------                                         % section divider
    disparityMap = disparitySGM(I1, I2, ...                                 % estimate dense disparity map
        'DisparityRange', dispRange, ...                                    % use requested disparity interval
        'UniquenessThreshold', 15);                                         % reject ambiguous matches somewhat

    % -----------------------------                                         % section divider
    % Build valid mask                                                      % keep only disparities we trust
    % -----------------------------                                         % section divider
    validMask = isfinite(disparityMap) & ...                                % keep finite disparities
                disparityMap > max(dispRange(1), minValidDisparity);        % keep disparities above minimum threshold

    % Suppress bright background regions                                    % remove white background pixels
    bgMask = (I1_input > backgroundThreshold);                              % mark bright left-image pixels as background
    validMask = validMask & ~bgMask;                                        % remove those pixels from valid set

    % -----------------------------                                         % section divider
    % Reconstruct in LEFT camera coordinates                                % convert disparity to 3D coordinates
    % For a parallel stereo pair:                                           % standard stereo formulas
    %   Z = fx * B / d                                                      % depth formula
    %   X = (u - cx) * Z / fx                                               % x-coordinate formula
    %   Y = (v - cy) * Z / fy                                               % y-coordinate formula
    % -----------------------------                                         % section divider
    B = norm(rig.C_right - rig.C_left);                                     % compute stereo baseline length

    [U, V] = meshgrid(1:W, 1:H);                                            % pixel coordinate grids
    D = double(disparityMap);                                               % convert disparity map to double

    Z = rig.fx * B ./ D;                                                    % compute depth from disparity
    X = (U - rig.cx) .* Z / rig.fx;                                         % compute x in left camera frame
    Y = (V - rig.cy) .* Z / rig.fy;                                         % compute y in left camera frame

    validMask = validMask & isfinite(X) & isfinite(Y) & isfinite(Z) & ...  % keep only finite reconstructed points
                (Z > 0) & (Z < maxDepth);                                   % keep only positive, not-too-far depth

    ptsLeft = [X(validMask), Y(validMask), Z(validMask)];                   % gather valid 3D points in left camera frame

    % Approximate mapping back to world coordinates                         % move 3D points into approximate world frame
    % rig.R_left_wc maps world -> left camera                               % explain current convention
    % so world approx = R' * cam + C_left                                   % inverse-like mapping formula
    R_cw = rig.R_left_wc';                                                  % transpose rotation to map camera -> world approx
    ptsWorldApprox = (R_cw * ptsLeft')' + rig.C_left;                       % map each point into world coordinates

    % Colors from original left image                                       % color the points using left image intensity
    grayVals = I1_input(validMask);                                         % grab grayscale values at valid pixels
    colors = repmat(uint8(grayVals), 1, 3);                                 % copy grayscale into RGB triplets

    ptCloud = pointCloud(ptsWorldApprox, 'Color', colors);                  % create MATLAB point cloud object

    % -----------------------------                                         % section divider
    % Outputs                                                               % pack everything into output struct
    % -----------------------------                                         % section divider
    recon = struct();                                                       % create output struct
    recon.left_gray_input        = I1_input;                                % store original left image
    recon.right_gray_input       = I2_input;                                % store original right image
    recon.left_gray_preprocessed = I1;                                      % store processed left image
    recon.right_gray_preprocessed = I2;                                     % store processed right image
    recon.disparity_map          = disparityMap;                            % store disparity result
    recon.valid_mask             = validMask;                               % store valid mask
    recon.points_left_camera     = ptsLeft;                                 % store 3D points in left camera frame
    recon.points_world_approx    = ptsWorldApprox;                          % store 3D points in approximate world frame
    recon.ptCloud                = ptCloud;                                 % store point cloud object
    recon.baseline               = B;                                       % store baseline length

    % -----------------------------                                         % section divider
    % Summary                                                               % print summary information
    % -----------------------------                                         % section divider
    if verbose                                                               % if verbose output is requested
        fprintf('\n=== RECONSTRUCT POINT CLOUD SUMMARY ===\n');             % print header
        fprintf('Mode: parallel raw stereo (no extra rectification)\n');    % print reconstruction mode
        fprintf('Image size: [%d, %d]\n', H, W);                            % print image size
        fprintf('Disparity range: [%.3f, %.3f]\n', dispRange(1), dispRange(2)); % print disparity interval
        fprintf('Baseline: %.6f\n', B);                                     % print baseline
        fprintf('Adaptive histogram equalization: %d\n', useAdaptiveHistEq); % print preprocessing flag
        fprintf('Valid disparity pixels: %d\n', nnz(validMask));            % print count of valid pixels
        fprintf('Recovered 3D points:    %d\n', size(ptsWorldApprox,1));    % print count of recovered 3D points

        if ~isempty(ptsWorldApprox)                                         % if any points were reconstructed
            pmin = min(ptsWorldApprox, [], 1);                              % compute min xyz values
            pmax = max(ptsWorldApprox, [], 1);                              % compute max xyz values
            fprintf('\nApprox world-coordinate point cloud bbox:\n');       % print bbox header
            fprintf('  x: [% .6f, % .6f]\n', pmin(1), pmax(1));             % print x-range
            fprintf('  y: [% .6f, % .6f]\n', pmin(2), pmax(2));             % print y-range
            fprintf('  z: [% .6f, % .6f]\n', pmin(3), pmax(3));             % print z-range
        end
        fprintf('=======================================\n\n');             % print footer
    end

    % -----------------------------                                         % section divider
    % Optional save                                                         % save point cloud if requested
    % -----------------------------                                         % section divider
    if savePointCloud                                                        % if saving is enabled
        scriptRoot = fileparts(fileparts(mfilename('fullpath')));           % get project root
        outDir = fullfile(scriptRoot, 'pointclouds');                       % choose pointcloud output folder
        if ~exist(outDir, 'dir')                                            % if folder does not exist
            mkdir(outDir);                                                  % create it
        end
        pcwrite(ptCloud, fullfile(outDir, pointCloudFile));                 % save point cloud to disk
    end

    % -----------------------------                                         % section divider
    % Visualization                                                         % display diagnostic figures
    % -----------------------------                                         % section divider
    if makeFigure                                                            % if display is requested
        fig1 = figure('Color','w','Name','Raw Stereo + Disparity');         % create figure for images
        tiledlayout(fig1, 2, 3, 'Padding', 'compact', 'TileSpacing', 'compact'); % 2x3 compact layout

        nexttile;                                                           % move to tile 1
        imshow(I1_input);                                                   % show original left image
        title('Left Input');                                                % title tile 1

        nexttile;                                                           % move to tile 2
        imshow(I2_input);                                                   % show original right image
        title('Right Input');                                               % title tile 2

        nexttile;                                                           % move to tile 3
        imagesc(disparityMap);                                              % show disparity map as color image
        axis image off;                                                     % preserve aspect and hide axes
        colorbar;                                                           % show color scale
        title('Disparity Map');                                             % title tile 3

        nexttile;                                                           % move to tile 4
        imshow(I1);                                                         % show processed left image
        title('Left Preprocessed');                                         % title tile 4

        nexttile;                                                           % move to tile 5
        imshow(I2);                                                         % show processed right image
        title('Right Preprocessed');                                        % title tile 5

        nexttile;                                                           % move to tile 6
        imshow(validMask);                                                  % show valid disparity mask
        title('Valid Mask');                                                % title tile 6

        fig2 = figure('Color','w','Name','Reconstructed Point Cloud');      % create figure for point cloud
        if ~isempty(ptsWorldApprox)                                         % if we have reconstructed points
            pcshow(ptCloud);                                                % display point cloud
            xlabel('X');                                                    % label x-axis
            ylabel('Y');                                                    % label y-axis
            zlabel('Z');                                                    % label z-axis
            title('Reconstructed Point Cloud (Approx World Coordinates)');  % title point cloud figure
        else                                                                % if no points survived
            axis off;                                                       % hide axes
            text(0.5, 0.5, 'No valid 3D points recovered', ...              % place fallback message
                'HorizontalAlignment', 'center');                           % center the message
        end
    end
end