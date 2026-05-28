function reconMerged = merge_reconstructed_clouds(reconA, reconB, varargin)
%MERGE_RECONSTRUCTED_CLOUDS Concatenate two reconstruction outputs and optionally grid-downsample.
%
% Required inputs:
%   reconA, reconB -> structs that contain:
%       recon.points_world_approx
%       recon.ptCloud
%
% Optional name-value pairs:
%   'VoxelSize'       -> grid size for optional dedup/downsample (meters)
%   'MakeFigure'      -> show merged cloud
%   'Verbose'         -> print summary
%   'KeepSourceViews' -> keep original view-A / view-B point arrays in output
%
% Notes:
%   - This version is search-safe by default:
%       MakeFigure = false
%       Verbose    = false

    p = inputParser;
    addParameter(p, 'VoxelSize', 0.01, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'MakeFigure', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Verbose', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'KeepSourceViews', false, @(x) islogical(x) || isnumeric(x));
    parse(p, varargin{:});

    voxelSize       = double(p.Results.VoxelSize);
    makeFigure      = logical(p.Results.MakeFigure);
    verbose         = logical(p.Results.Verbose);
    keepSourceViews = logical(p.Results.KeepSourceViews);

    if ~isfield(reconA, 'points_world_approx') || ~isfield(reconA, 'ptCloud')
        error('reconA must contain points_world_approx and ptCloud.');
    end
    if ~isfield(reconB, 'points_world_approx') || ~isfield(reconB, 'ptCloud')
        error('reconB must contain points_world_approx and ptCloud.');
    end

    ptsA = reconA.points_world_approx;
    ptsB = reconB.points_world_approx;
    colsA = reconA.ptCloud.Color;
    colsB = reconB.ptCloud.Color;

    if isempty(ptsA) || isempty(ptsB)
        error('One of the input reconstructions is empty, so merge is not meaningful.');
    end

    if size(ptsA,2) ~= 3 || size(ptsB,2) ~= 3
        error('Input point arrays must be N-by-3.');
    end

    if size(colsA,1) ~= size(ptsA,1) || size(colsB,1) ~= size(ptsB,1)
        error('Point/color count mismatch in one of the input reconstructions.');
    end

    ptsRaw = [ptsA; ptsB];
    colsRaw = [colsA; colsB];
    rawMergedCount = size(ptsRaw, 1);

    pcRaw = pointCloud(ptsRaw, 'Color', colsRaw);

    if voxelSize > 0 && exist('pcdownsample', 'file') == 2
        pcMerged = pcdownsample(pcRaw, 'gridAverage', voxelSize);
        ptsMerged = pcMerged.Location;
        colsMerged = pcMerged.Color;
    else
        pcMerged = pcRaw;
        ptsMerged = ptsRaw;
        colsMerged = colsRaw;
    end

    reconMerged = struct();
    reconMerged.points_world_approx = ptsMerged;
    reconMerged.ptCloud = pointCloud(ptsMerged, 'Color', colsMerged);
    reconMerged.raw_merged_point_count = rawMergedCount;
    reconMerged.final_merged_point_count = size(ptsMerged, 1);
    reconMerged.voxel_size = voxelSize;

    if keepSourceViews
        reconMerged.points_world_approx_a = ptsA;
        reconMerged.points_world_approx_b = ptsB;
    end

    if verbose
        fprintf('\n=== MERGED RECONSTRUCTION SUMMARY ===\n');
        fprintf('View A points:             %d\n', size(ptsA,1));
        fprintf('View B points:             %d\n', size(ptsB,1));
        fprintf('Raw merged points:         %d\n', rawMergedCount);
        fprintf('Final merged points:       %d\n', size(ptsMerged,1));
        fprintf('Voxel size used (m):       %.6f\n', voxelSize);
        fprintf('=====================================\n\n');
    end

    if makeFigure
        fig = figure('Color', 'w', 'Name', 'Merged Reconstructed Point Cloud');
        figCleanup = onCleanup(@() local_close_valid_figure(fig)); %#ok<NASGU>

        pcshow(reconMerged.ptCloud);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title('Merged Reconstructed Point Cloud');

        drawnow;
    end
end

function local_close_valid_figure(fig)
    if ~isempty(fig) && isgraphics(fig, 'figure')
        delete(fig);
        drawnow;
    end
end