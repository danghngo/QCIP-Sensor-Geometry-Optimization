function reportPath = save_jason_ship_metrics_report(stats, metrics, rig, sceneInfo, varargin) % Save a timestamped text report of tanker reconstruction metrics.
%SAVE_JASON_SHIP_METRICS_REPORT Write a clean .txt summary of one tanker run.

    p = inputParser;                                                       % Create parser for optional arguments.
    addParameter(p, 'OutDir', fullfile(fileparts(fileparts(mfilename('fullpath'))), 'metrics'), @(x) ischar(x) || isstring(x)); % Default metrics folder.
    addParameter(p, 'RunLabel', 'tanker_run', @(x) ischar(x) || isstring(x)); % Default short run label.
    addParameter(p, 'TestScript', 'test_jason_ship_upgraded_metrics.m', @(x) ischar(x) || isstring(x)); % Default test script name.
    addParameter(p, 'Notes', '', @(x) ischar(x) || isstring(x));           % Optional extra notes.
    addParameter(p, 'TimestampTag', '', @(x) ischar(x) || isstring(x));    % Optional shared timestamp string.
    parse(p, varargin{:});                                                 % Parse user inputs.

    outDir = char(p.Results.OutDir);                                       % Store output folder as char.
    runLabel = char(p.Results.RunLabel);                                   % Store run label as char.
    testScript = char(p.Results.TestScript);                               % Store test script name as char.
    notes = char(p.Results.Notes);                                         % Store notes as char.
    timestampTag = char(p.Results.TimestampTag);                           % Store timestamp tag as char.

    if isempty(timestampTag)                                               % If caller did not provide a timestamp tag,
        timestampTag = char(string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))); % create one here.
    end

    if ~exist(outDir, 'dir')                                               % If output folder does not exist,
        mkdir(outDir);                                                     % create it.
    end

    fileName = sprintf('%s_%s.txt', runLabel, timestampTag);               % Build output filename.
    reportPath = fullfile(outDir, fileName);                               % Build full output path.

    fid = fopen(reportPath, 'w');                                          % Open report file for writing.
    if fid < 0                                                             % If file could not be opened,
        error('Could not open report file for writing: %s', reportPath);   % stop with error.
    end

    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>                   % Ensure file closes even if an error occurs.

    fprintf(fid, '============================================================\n'); % Write top border.
    fprintf(fid, 'JASON SHIP SYNTHETIC STEREO METRICS REPORT\n');          % Write report title.
    fprintf(fid, '============================================================\n'); % Write border.
    fprintf(fid, 'Timestamp:   %s\n', timestampTag);                       % Write shared timestamp.
    fprintf(fid, 'Run label:   %s\n', runLabel);                           % Write run label.
    fprintf(fid, 'Test script: %s\n', testScript);                         % Write driver script name.

    if isfield(sceneInfo, 'ship_script')                                   % If sceneInfo stores ship script path,
        fprintf(fid, 'Ship script: %s\n', sceneInfo.ship_script);          % write it.
    end

    if ~isempty(notes)                                                     % If user supplied notes,
        fprintf(fid, 'Notes:      %s\n', notes);                           % write them.
    end

    fprintf(fid, '\n');                                                    % Blank line.

    fprintf(fid, '---------------- RENDER / RIG PARAMETERS ----------------\n'); % Write section header.
    fprintf(fid, 'AzimuthDeg:        %.6f\n', rig.azimuth_deg);            % Write azimuth.
    fprintf(fid, 'ElevationDeg:      %.6f\n', rig.elevation_deg);          % Write elevation.
    fprintf(fid, 'Range:             %.6f\n', rig.range);                  % Write camera range.
    fprintf(fid, 'Baseline:          %.6f\n', rig.baseline);               % Write stereo baseline.
    fprintf(fid, 'HFOVDeg:           %.6f\n', rig.hfov_deg);               % Write horizontal FOV.
    fprintf(fid, 'ImageSize:         [%d %d]\n', rig.image_size(1), rig.image_size(2)); % Write image size.
    fprintf(fid, 'fx:                %.6f\n', rig.fx);                     % Write focal length fx.
    fprintf(fid, 'fy:                %.6f\n', rig.fy);                     % Write focal length fy.
    fprintf(fid, 'cx:                %.6f\n', rig.cx);                     % Write principal point cx.
    fprintf(fid, 'cy:                %.6f\n', rig.cy);                     % Write principal point cy.
    fprintf(fid, '\n');                                                    % Blank line.

    fprintf(fid, '---------------- POINT-CLOUD ANALYSIS --------------------\n'); % Write stats section header.
    fprintf(fid, 'Raw point count:        %d\n', metrics.raw_point_count); % Write raw point count.
    fprintf(fid, 'Cropped point count:    %d\n', metrics.cropped_point_count); % Write cropped point count.
    fprintf(fid, 'Keep fraction:          %.6f\n', metrics.keep_fraction); % Write keep fraction.

    fprintf(fid, 'Raw bbox min:           [%.6f %.6f %.6f]\n', stats.raw_bbox_min(1), stats.raw_bbox_min(2), stats.raw_bbox_min(3)); % Write raw bbox min.
    fprintf(fid, 'Raw bbox max:           [%.6f %.6f %.6f]\n', stats.raw_bbox_max(1), stats.raw_bbox_max(2), stats.raw_bbox_max(3)); % Write raw bbox max.
    fprintf(fid, 'Raw centroid:           [%.6f %.6f %.6f]\n', stats.raw_centroid(1), stats.raw_centroid(2), stats.raw_centroid(3)); % Write raw centroid.

    fprintf(fid, 'Crop bbox min:          [%.6f %.6f %.6f]\n', stats.crop_bbox_min(1), stats.crop_bbox_min(2), stats.crop_bbox_min(3)); % Write cropped bbox min.
    fprintf(fid, 'Crop bbox max:          [%.6f %.6f %.6f]\n', stats.crop_bbox_max(1), stats.crop_bbox_max(2), stats.crop_bbox_max(3)); % Write cropped bbox max.
    fprintf(fid, 'Crop centroid:          [%.6f %.6f %.6f]\n', stats.crop_centroid(1), stats.crop_centroid(2), stats.crop_centroid(3)); % Write cropped centroid.

    fprintf(fid, 'CropX used:             [%.6f %.6f]\n', stats.crop_x(1), stats.crop_x(2)); % Write crop X.
    fprintf(fid, 'CropY used:             [%.6f %.6f]\n', stats.crop_y(1), stats.crop_y(2)); % Write crop Y.
    fprintf(fid, 'CropZ used:             [%.6f %.6f]\n', stats.crop_z(1), stats.crop_z(2)); % Write crop Z.
    fprintf(fid, '\n');                                                    % Blank line.

    fprintf(fid, '---------------- GT MESH / METRICS -----------------------\n'); % Write GT metrics section header.
    fprintf(fid, 'GT mesh vertices:             %d\n', metrics.gt_vertex_count); % Write GT vertex count.
    fprintf(fid, 'GT mesh triangles:            %d\n', metrics.gt_triangle_count); % Write GT triangle count.
    fprintf(fid, 'Total GT area:                %.6f\n', metrics.total_gt_area); % Write total GT area.
    fprintf(fid, 'Evaluated recon points:       %d\n', metrics.eval_point_count); % Write evaluated reconstruction count.

    fprintf(fid, 'Proxy one-sided RMSE (m):     %.6f\n', metrics.proxy_one_sided_rmse); % Write proxy one-sided RMSE.
    fprintf(fid, 'Proxy symmetric RMSE (m):     %.6f\n', metrics.proxy_symmetric_rmse); % Write proxy symmetric RMSE.
    fprintf(fid, 'Centroid one-sided RMSE (m):  %.6f\n', metrics.centroid_one_sided_rmse); % Write centroid one-sided RMSE.
    fprintf(fid, 'Centroid symmetric RMSE (m):  %.6f\n', metrics.centroid_symmetric_rmse); % Write centroid symmetric RMSE.
    fprintf(fid, 'Mesh-surface one-sided RMSE:  %.6f\n', metrics.mesh_surface_one_sided_rmse); % Write mesh-surface RMSE.
    fprintf(fid, '\n');                                                    % Blank line.

    fprintf(fid, '---------------- COVERAGE -------------------------------\n'); % Write coverage section header.
    fprintf(fid, 'Covered GT triangles <= 0.05 m:   %d\n', metrics.covered_gt_triangle_count_005); % Write GT triangle count coverage at 5 cm.
    fprintf(fid, 'Covered GT triangles <= 0.10 m:   %d\n', metrics.covered_gt_triangle_count_010); % Write GT triangle count coverage at 10 cm.
    fprintf(fid, 'Covered GT triangles <= 0.20 m:   %d\n', metrics.covered_gt_triangle_count_020); % Write GT triangle count coverage at 20 cm.
    fprintf(fid, 'Covered GT triangle frac <= 0.05: %.6f\n', metrics.covered_gt_triangle_fraction_005); % Write GT triangle fraction at 5 cm.
    fprintf(fid, 'Covered GT triangle frac <= 0.10: %.6f\n', metrics.covered_gt_triangle_fraction_010); % Write GT triangle fraction at 10 cm.
    fprintf(fid, 'Covered GT triangle frac <= 0.20: %.6f\n', metrics.covered_gt_triangle_fraction_020); % Write GT triangle fraction at 20 cm.

    fprintf(fid, 'Covered GT area <= 0.05 m:       %.6f\n', metrics.covered_gt_area_005); % Write GT area covered at 5 cm.
    fprintf(fid, 'Covered GT area <= 0.10 m:       %.6f\n', metrics.covered_gt_area_010); % Write GT area covered at 10 cm.
    fprintf(fid, 'Covered GT area <= 0.20 m:       %.6f\n', metrics.covered_gt_area_020); % Write GT area covered at 20 cm.
    fprintf(fid, 'Covered GT area frac <= 0.05:    %.6f\n', metrics.covered_gt_area_fraction_005); % Write GT area fraction at 5 cm.
    fprintf(fid, 'Covered GT area frac <= 0.10:    %.6f\n', metrics.covered_gt_area_fraction_010); % Write GT area fraction at 10 cm.
    fprintf(fid, 'Covered GT area frac <= 0.20:    %.6f\n', metrics.covered_gt_area_fraction_020); % Write GT area fraction at 20 cm.
    fprintf(fid, '\n');                                                    % Blank line.

    fprintf(fid, '---------------- DENSITY --------------------------------\n'); % Write density section header.
    fprintf(fid, 'Cropped pts / total GT area:        %.6f\n', metrics.cropped_points_per_total_gt_area); % Write density over total GT area.
    fprintf(fid, 'Cropped pts / covered GT area 0.05: %.6f\n', metrics.cropped_points_per_covered_gt_area_005); % Write density over covered GT area at 5 cm.
    fprintf(fid, 'Cropped pts / covered GT area 0.10: %.6f\n', metrics.cropped_points_per_covered_gt_area_010); % Write density over covered GT area at 10 cm.
    fprintf(fid, 'Cropped pts / covered GT area 0.20: %.6f\n', metrics.cropped_points_per_covered_gt_area_020); % Write density over covered GT area at 20 cm.
    fprintf(fid, '\n');                                                    % Blank line.

    fprintf(fid, '---------------- QUICK TAKEAWAY --------------------------\n'); % Write interpretation section header.
    fprintf(fid, 'One-sided metrics emphasize accuracy of reconstructed points.\n'); % Explain one-sided metrics.
    fprintf(fid, 'Symmetric metrics emphasize both accuracy and completeness.\n');   % Explain symmetric metrics.
    fprintf(fid, 'Coverage metrics measure how much of the GT mesh is near the reconstruction.\n'); % Explain coverage metrics.
    fprintf(fid, 'Density metrics measure how densely the useful cropped reconstruction samples the GT.\n'); % Explain density metrics.
    fprintf(fid, '============================================================\n'); % Write closing border.
end