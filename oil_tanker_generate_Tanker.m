%% oil_tanker_generate_Tanker.m  —  CONTAINER SHIP VERSION                 % script title
% =========================================================                 % divider
% BUILD A 3D CONTAINER SHIP MODEL                                          % script purpose
% =========================================================                 % divider
% Same hull form as the tanker (Bezier waterplane oval),                   % describe hull basis
% but the cargo deck is replaced with colourful stacked                    % describe cargo change
% shipping containers — ideal for KAZE feature matching.                   % explain why visually rich
%                                                                          % blank comment line
% Variables exported to workspace (used by later scripts):                 % list exported variables
%   x_ship, y_rail  – hull waterplane profile                              % hull profile arrays
%   zdeck, zkeel    – deck and keel heights                                % key z-heights
%   depth           – hull depth                                           % hull thickness
% =========================================================                 % divider
clc;                                                                       % clear Command Window

global ztop                                                                 % make ztop global
ztop  = 0.5;                                                               % define base top reference

zdeck = ztop + 0.52;      % flat deck height  = 1.02                       % compute deck height
zkeel = ztop - 0.10;      % flat keel height  = 0.40                       % compute keel height
depth = zdeck - zkeel;    % total hull depth  = 0.62                       % compute hull depth

figure('Color', 'w', 'Position', [100 100 1400 800]);                      % create main figure window

% =========================================================                 % divider
% WATERPLANE OVAL  (identical Bezier hull to the tanker)                   % define top-view hull curve
%   Length = 8.4 m, half-beam = 0.60 m                                     % target ship dimensions
% =========================================================                 % divider
nx = 45;                                                                   % number of x-profile samples
n1 = round(nx * 0.20);                                                     % number of Bezier samples in first segment
n2 = nx - n1 - 1;                                                          % number of Bezier samples in second segment

p1 = [0   0    zdeck];                                                     % bow/deck start point
p2 = [4.2 0.60 zdeck];                                                     % midship control point
p3 = [8.4 0    zdeck];                                                     % stern/deck end point

alfa1 = 0.30;  alfa2 = 3.50;                                               % Bezier handle lengths
gamma1 = 3.50; gamma2 = 0.30;                                              % Bezier handle lengths
k1 = [4 3 0];  k2 = [1 0 0];  k3 = [-4 3 0];                              % Bezier handle directions

b1 = p1 + alfa1 * k1 / norm(k1);   c1 = p2 + gamma1 * (-k2);              % first segment control points
b2 = p2 + alfa2 * k2;              c2 = p3 + gamma2 * k3 / norm(k3);      % second segment control points

rpos = [bezier(p1, b1, c1, p2, n1);                                        % sample first Bezier segment
        bezier(p2, b2, c2, p3, n2);                                        % sample second Bezier segment
        p3];                                                                % append exact endpoint

x_ship = rpos(:, 1);                                                       % extract x profile of hull edge
y_rail = rpos(:, 2);                                                       % extract half-beam profile of hull edge

% =========================================================                 % divider
% HULL  (vertical sides + flat bottom + flat deck)                         % build hull surfaces
% =========================================================                 % divider
nz_s = 16;                                                                 % number of vertical samples along hull sides
z_side = linspace(zkeel, zdeck, nz_s);                                     % evenly spaced side z-values

X_ps = repmat(x_ship', nz_s, 1);                                           % x-grid for one hull side
Y_ps = repmat(y_rail', nz_s, 1);                                           % y-grid for one hull side
Z_ps = repmat(z_side',  1,   nx);                                          % z-grid for one hull side

surfl(X_ps,  Y_ps, Z_ps);  hold on                                         % draw port-side hull surface and hold figure
surfl(X_ps, -Y_ps, Z_ps);                                                  % draw starboard-side hull surface

% ── Fix proportions immediately so the ship never appears "fat" ──        % explain axis setup
% axis equal must be set before any mid-script figure switches             % explain timing issue
% (e.g. the nameplate off-screen figure) trigger a re-render.              % explain why this matters
axis([0 9 -1.5 1.5 0 3.5]);                                                % set axis limits for display
axis equal;                                                                % force equal scaling in x/y/z
axis vis3d;                                                                % preserve 3D aspect while rotating
camproj('perspective');                                                    % use perspective projection
drawnow;                                                                   % force graphics refresh now

ny_b = 22;                                                                 % number of samples across beam for bottom/deck
Y_frac = linspace(-1, 1, ny_b)';                                           % normalized beam positions
X_bt   = repmat(x_ship', ny_b, 1);                                         % x-grid for bottom/deck surfaces
Y_bt   = Y_frac * y_rail';                                                 % y-grid scaled by hull half-beam
Z_bt   = zkeel * ones(ny_b, nx);                                           % z-grid for flat bottom

surfl(X_bt, Y_bt, Z_bt);                        % flat bottom              % draw flat hull bottom
surfl(X_bt, Y_bt, zdeck * ones(ny_b, nx));      % flat deck               % draw flat top deck

% =========================================================                 % divider
% WATER SURFACE                                                            % draw waterline/water hints
% =========================================================                 % divider
waterline = zkeel + 0.54 * depth;                                          % define waterline height

for j = 0:0.15:8.4                                                         % sweep along ship length
    [~, idx] = min(abs(x_ship - j));                                       % find nearest hull profile sample
    yw = y_rail(idx);                                                      % get half-width at that x
    if yw < 0.04, continue, end                                            % skip extremely narrow tip regions
    wx = [];  wy = [];  wz = [];                                           % initialize temporary water-point arrays
    for i = -yw:0.06:yw                                                    % sweep across beam at this x
        wx = [wx  j];                                                      % append x coordinate
        wy = [wy  i];                                                      % append y coordinate
        wz = [wz  0.005*sin(j*10) + 0.005*sin(i*10) + waterline];          % append slightly wavy water z
    end
    plot3(wx,  wy, wz, 'b');                                               % draw water trace on one side
    plot3(wx, -wy, wz, 'b');                                               % mirror water trace on other side
end

for j = 0:0.12:8.4                                                         % sweep along ship length again
    [~, idx] = min(abs(x_ship - j));                                       % find nearest hull sample
    yw = y_rail(idx);                                                      % get half-width there
    if yw < 0.04, continue, end                                            % skip very narrow ends
    plot3([j j], [ yw  yw], [waterline-0.03 waterline+0.05], 'g-', 'LineWidth', 2); % draw green vertical marker port side
    plot3([j j], [-yw -yw], [waterline-0.03 waterline+0.05], 'g-', 'LineWidth', 2); % draw green vertical marker starboard side
end

% =========================================================                 % divider
% STERN SUPERSTRUCTURE  (bridge house – same as tanker)                    % build bridge block
% =========================================================                 % divider
x0 = 1.34;  x1 = 1.84;  yw_s = 0.21;                                       % x-span and half-width of bridge block
z0 = zdeck; z1 = zdeck + 0.52;                                             % bottom and top z of bridge block
nw = 10;    xc_s = (x0 + x1) / 2;                                          % mesh resolution and x-center

Y_w   = linspace(-yw_s, yw_s, nw);                                         % y grid along bridge width
Z_w   = linspace(z0, z1, nw);                                              % z grid along bridge height
X_len = linspace(x0, x1, nw);                                              % x grid along bridge length

[Yw,  Zw]  = meshgrid(Y_w,   Z_w);                                         % grid for front/aft walls
[Xw,  Zw2] = meshgrid(X_len, Z_w);                                         % grid for side walls

surfl(x0 * ones(nw,nw),  Yw,  Zw);        % front wall                     % draw front wall
surfl(x1 * ones(nw,nw),  Yw,  Zw);        % aft wall                       % draw aft wall
surfl(Xw,  yw_s * ones(nw,nw), Zw2);      % port wall                      % draw port wall
surfl(Xw, -yw_s * ones(nw,nw), Zw2);      % starboard wall                 % draw starboard wall

[Xr, Yr] = meshgrid(X_len, Y_w);                                             % grid for roof
surfl(Xr, Yr, z1 * ones(nw,nw));          % roof                            % draw roof surface

% Bridge windows                                                           % start adding windows
zw0 = z1 - 0.20;  zw1 = z1 - 0.06;                                         % lower and upper window z
for yw_w = -0.13:0.10:0.13                                                 % place several windows across width
    fill3([x0-0.01 x0-0.01 x0-0.01 x0-0.01], ...                           % x coordinates of a window quad
          [yw_w-0.04 yw_w+0.04 yw_w+0.04 yw_w-0.04], ...                   % y coordinates of a window quad
          [zw0 zw0 zw1 zw1], [0.45 0.70 1.00]);                            % z coordinates and window color
end

% =========================================================                 % divider
% FUNNEL                                                                   % build funnel/cap geometry
% =========================================================                 % divider
nf = 18;  rf = 0.08;                                                       % number of circle samples and funnel radius
t  = 0 : 2*pi/nf : 2*pi;                                                   % angular samples around circle
xf = rf*cos(t) + xc_s;  yf = rf*sin(t);                                    % circular cross-section centered near bridge

xfm = [];  yfm = [];  zfm = [];                                            % initialize funnel surface arrays
for h = z1 : 0.03 : z1+0.38                                                % sweep upward in z
    xfm = [xfm; xf];  yfm = [yfm; yf];                                     % append one circular ring
    zfm = [zfm; h * ones(size(xf))];                                       % append matching z ring
end
surfl(xfm, yfm, zfm);                                                      % draw funnel wall

rc  = 0.11;  xc2 = rc*cos(t) + xc_s;  yc2 = rc*sin(t);                     % larger radius for funnel cap
xcm = [];  ycm = [];  zcm = [];                                            % initialize cap arrays
for h = z1+0.33 : 0.02 : z1+0.42                                           % sweep cap height
    xcm = [xcm; xc2];  ycm = [ycm; yc2];                                   % append cap ring
    zcm = [zcm; h * ones(size(xc2))];                                      % append cap z ring
end
surfl(xcm, ycm, zcm);                                                      % draw funnel cap

% =========================================================                 % divider
% BOW GANTRY CRANE  (characteristic of container ships)                    % build bow crane
% =========================================================                 % divider
xa_c = 7.50;                                                               % x-position of gantry crane
% Vertical mast                                                            % comment for vertical mast
plot3([xa_c xa_c], [0 0], [zdeck zdeck+0.50], 'k-', 'LineWidth', 5);       % draw crane mast
% Cross-arm                                                                % comment for cross-arm
plot3([xa_c-0.28 xa_c+0.28], [0 0], [zdeck+0.50 zdeck+0.50], 'k-', 'LineWidth', 3); % draw crane top bar
% Diagonal braces                                                          % comment for braces
plot3([xa_c xa_c-0.28], [0 0], [zdeck zdeck+0.50], 'k-', 'LineWidth', 2);  % draw left brace
plot3([xa_c xa_c+0.28], [0 0], [zdeck zdeck+0.50], 'k-', 'LineWidth', 2);  % draw right brace
% Hoist cables                                                             % comment for cables
plot3([xa_c-0.14 xa_c-0.14], [0 0], [zdeck+0.50 zdeck+0.02], 'k-', 'LineWidth', 1); % draw left cable
plot3([xa_c+0.14 xa_c+0.14], [0 0], [zdeck+0.50 zdeck+0.02], 'k-', 'LineWidth', 1); % draw right cable

% =========================================================                 % divider
% CONTAINER STACKS                                                         % build colorful cargo stacks
%   6 vivid colours, fixed random seed → reproducible layout               % explain repeatability
%   4 columns × 10 rows × 2 layers = 80 containers                         % explain layout size
% =========================================================                 % divider
rng(42);   % fixed seed: same colour layout every run                      % set random seed for reproducible colors

cont_colors = {[0.88 0.12 0.12], ...   % red                               % define red container color
               [0.12 0.32 0.90], ...   % blue                              % define blue container color
               [0.96 0.74 0.04], ...   % yellow                            % define yellow container color
               [0.94 0.40 0.05], ...   % orange                            % define orange container color
               [0.12 0.68 0.20], ...   % green                             % define green container color
               [0.58 0.14 0.72]};      % purple                            % define purple container color

cL  = 0.58;    % container length along X  (ship length direction)         % set container length
cW  = 0.24;    % container width  along Y  (across ship)                   % set container width
cH  = 0.27;    % container height along Z                                  % set container height
gap = 0.015;   % gap between adjacent containers                           % set spacing between containers

x_start = x1 + 0.12;   % just forward of superstructure  (~1.96)           % container deck start x
x_end   = 8.00;                                                            % container deck end x

% 4 columns centred on deck                                                % explain column placement
n_cols    = 4;                                                             % number of container columns
y_centers = (-(n_cols-1)/2 : 1 : (n_cols-1)/2) * (cW + gap);              % y-centers of each column
% ≈ [-0.383, -0.128, +0.128, +0.383]                                       % approximate y values

n_rows   = floor((x_end - x_start) / (cL + gap));   % ≈ 10                % number of rows that fit
n_layers = 2;                                         % stack 2 high       % number of vertical layers

for rr = 1 : n_rows                                                         % loop over rows along ship length
    x0c = x_start + (rr-1)*(cL + gap);                                     % front x of current container row
    x1c = x0c + cL;                                                        % back x of current container row
    if x1c > x_end, break; end                                             % stop if row would exceed deck length

    for cc = 1 : n_cols                                                    % loop over columns across width
        yc  = y_centers(cc);                                               % center y of current column
        y0c = yc - cW/2;                                                   % lower y of container box
        y1c = yc + cW/2;                                                   % upper y of container box

        for lyr = 1 : n_layers                                             % loop over vertical layers
            z0c = zdeck + (lyr-1)*(cH + gap);                              % bottom z of container
            z1c = z0c + cH;                                                % top z of container

            col = cont_colors{randi(numel(cont_colors))};                  % randomly choose one of the preset colors

            % --- 6 faces of the container box ---                         % explain face drawing
            % Front  (X = x0c)                                             % front face comment
            fill3([x0c x0c x0c x0c], [y0c y1c y1c y0c], [z0c z0c z1c z1c], ... % front face vertices
                  col, 'EdgeColor', 'k', 'LineWidth', 0.5);                % draw front face
            % Back   (X = x1c)                                             % back face comment
            fill3([x1c x1c x1c x1c], [y0c y1c y1c y0c], [z0c z0c z1c z1c], ... % back face vertices
                  col, 'EdgeColor', 'k', 'LineWidth', 0.5);                % draw back face
            % Port   (Y = y0c)                                             % port side comment
            fill3([x0c x1c x1c x0c], [y0c y0c y0c y0c], [z0c z0c z1c z1c], ... % port face vertices
                  col, 'EdgeColor', 'k', 'LineWidth', 0.5);                % draw port face
            % Stbd   (Y = y1c)                                             % starboard side comment
            fill3([x0c x1c x1c x0c], [y1c y1c y1c y1c], [z0c z0c z1c z1c], ... % starboard face vertices
                  col, 'EdgeColor', 'k', 'LineWidth', 0.5);                % draw starboard face
            % Bottom (Z = z0c)                                             % bottom face comment
            fill3([x0c x1c x1c x0c], [y0c y0c y1c y1c], [z0c z0c z0c z0c], ... % bottom face vertices
                  col, 'EdgeColor', 'k', 'LineWidth', 0.5);                % draw bottom face
            % Top    (Z = z1c)                                             % top face comment
            fill3([x0c x1c x1c x0c], [y0c y0c y1c y1c], [z1c z1c z1c z1c], ... % top face vertices
                  col, 'EdgeColor', 'k', 'LineWidth', 0.5);                % draw top face
        end
    end
end

% =========================================================                 % divider
% RED ANTI-FOULING BAND at waterline                                       % draw red hull stripe
% =========================================================                 % divider
wl_lo = waterline - 0.08;                                                  % lower stripe z
wl_hi = waterline + 0.04;                                                  % upper stripe z
for jj = 1 : length(x_ship)-1                                              % loop over hull segments
    xa_ = x_ship(jj);   xb_ = x_ship(jj+1);                                % x values of segment ends
    ya_ = y_rail(jj);   yb_ = y_rail(jj+1);                                % y values of segment ends
    if ya_ < 0.04 || yb_ < 0.04, continue; end                             % skip tiny tip segments
    fill3([xa_ xb_ xb_ xa_], [ ya_  yb_  yb_  ya_], ...                    % port-side stripe quad
          [wl_lo wl_lo wl_hi wl_hi], [0.75 0.08 0.08], 'EdgeColor', 'none'); % draw red stripe port side
    fill3([xa_ xb_ xb_ xa_], [-ya_ -yb_ -yb_ -ya_], ...                    % starboard-side stripe quad
          [wl_lo wl_lo wl_hi wl_hi], [0.75 0.08 0.08], 'EdgeColor', 'none'); % draw red stripe starboard side
end

% Navigation lights (port = red, starboard = green)                        % add nav lights
t_c = linspace(0, 2*pi, 24);  r_c = 0.06;                                  % circle samples and light radius
fill3(r_c*cos(t_c) + 7.80,  r_c*sin(t_c) + 0.32, (zdeck+0.022)*ones(1,24), ... % port light circle
      [0.95 0.10 0.10], 'EdgeColor', 'k', 'LineWidth', 0.8);               % draw red port light
fill3(r_c*cos(t_c) + 7.80,  r_c*sin(t_c) - 0.32, (zdeck+0.022)*ones(1,24), ... % starboard light circle
      [0.10 0.80 0.15], 'EdgeColor', 'k', 'LineWidth', 0.8);               % draw green starboard light

% =========================================================                 % divider
% SHIP NAME PLATE  —  "JASON LUO"  (texture-mapped onto hull)              % build textured nameplate
% =========================================================                 % divider
% Strategy: render the text into a hidden off-screen figure,               % explain method
% capture it as a pixel image, then map that image onto a flat             % explain texture-map step
% surf() quad sitting exactly on the hull surface.                         % explain geometry target
% This means the text is genuine 3D geometry — it stays correctly          % explain benefit over screen text
% oriented on the hull when the model is rotated, unlike text()            % explain why not plain text()
% which is always screen-facing.                                           % finish explanation
% ---------------------------------------------------------                % divider
x_np0 = 3.0;   x_np1 = 6.4;    % nameplate X span (midship region)         % x-range of nameplate
z_np0 = 0.78;  z_np1 = 0.97;   % Z span: above red band, below deck        % z-range of nameplate

% Y position: interpolate hull half-beam at nameplate centre               % explain next computation
x_mid_np = (x_np0 + x_np1) / 2;                                            % midpoint x of nameplate
[~, idx_np] = min(abs(x_ship - x_mid_np));                                 % find nearest hull sample
y_np = y_rail(idx_np) * 1.004;  % push slightly outside hull face          % place nameplate slightly outside hull

% ── Step 1: render nameplate to a PNG file (robust) ──────────            % start off-screen text rendering
% getframe() is unreliable on Visible:'off' figures in some MATLAB         % explain why print is used
% versions — print() to a temp file is always stable.                      % explain robustness
main_fig = gcf;                                                            % remember main ship figure

fig_np = figure('Visible', 'off', 'Color', 'white', ...                    % create hidden figure for nameplate
                'Units', 'pixels', 'Position', [50 50 700 120]);           % set hidden figure size
axes('Position', [0 0 1 1]);                                               % create axes filling that figure
set(gca, 'XLim', [0 1], 'YLim', [0 1], 'Visible', 'off', 'Color', 'white'); % configure hidden axes
hold on;                                                                   % allow multiple graphics objects

% White background + dark-blue border                                      % describe plate background
fill([0 1 1 0 0], [0 0 1 1 0], 'white', ...                                % draw rectangular plate fill
     'EdgeColor', [0.10 0.10 0.50], 'LineWidth', 5);                       % set border color and width

% Main ship name                                                           % start main text
text(0.5, 0.63, 'JASON LUO', ...                                           % place name text in normalized coords
     'FontSize', 30, 'FontWeight', 'bold', ...                             % style the text
     'Color', [0.05 0.05 0.45], ...                                        % set dark-blue text color
     'HorizontalAlignment', 'center', ...                                  % center horizontally
     'VerticalAlignment', 'middle', ...                                    % center vertically
     'Units', 'normalized');                                               % interpret coords as normalized

% Sub-line (no TeX interpreter to avoid rendering issues)                  % describe subtitle
text(0.5, 0.16, 'M/V JASON LUO  -  IMO 0000001', ...                       % place subtitle text
     'FontSize', 9, 'Color', [0.35 0.35 0.35], ...                         % style subtitle
     'HorizontalAlignment', 'center', ...                                  % center horizontally
     'VerticalAlignment', 'middle', ...                                    % center vertically
     'Units', 'normalized');                                               % use normalized coords

tmp_np = fullfile(tempdir, 'nameplate_tmp.png');                           % choose temporary PNG path
print(fig_np, tmp_np, '-dpng', '-r96');                                    % render hidden figure to PNG file
close(fig_np);                                                             % close hidden nameplate figure
np_img = imread(tmp_np);                                                   % read generated PNG back into MATLAB
if exist(tmp_np, 'file'), delete(tmp_np); end                              % delete temporary file if it exists

% ── Step 2: curved surf following hull contour ───────────────            % start mapping nameplate onto hull
% KEY FIX for mirroring: surf() col-1 appears on the visual RIGHT          % explain mirrored texture issue
% when looking at the port side from outside.  Reversing X so that         % explain fix strategy
% col-1 = x_np1 (stern) makes the image left-edge ("J") land on            % explain why reversal works
% the visual LEFT — text reads correctly from both sides.                  % explain result
figure(main_fig); hold on;                                                 % return to main figure and keep drawing

n_np = 30;                                                                 % number of samples along nameplate span
x_np_fwd = linspace(x_np0, x_np1, n_np);   % 3.0 → 6.4                    % forward x samples along plate
y_np_fwd = zeros(1, n_np);                                                   % initialize corresponding hull y values
for ii = 1:n_np                                                            % loop across plate samples
    [~, idx_ii] = min(abs(x_ship - x_np_fwd(ii)));                         % find nearest hull sample
    y_np_fwd(ii) = y_rail(idx_ii) * 1.004;                                 % set nameplate y just outside hull
end

% PORT side: viewing from +Y, RIGHT = −X direction.                        % explain port-side mapping
%   Reversing X (6.4 → 3.0) places image col-1 ("J" side) on the visual LEFT. % explain reversal
x_np_r = x_np_fwd(end:-1:1);   % 6.4 → 3.0  (reversed for port)           % reversed x order for port
y_np_r = y_np_fwd(end:-1:1);                                               % reversed y order to match

% STARBOARD side: viewing from −Y, RIGHT = +X direction.                   % explain starboard mapping
%   Forward X (3.0 → 6.4) places image col-1 ("J" side) on the visual LEFT. % explain non-reversal
%   (opposite to port — that's why we need separate grids for each side)   % explain why two surfaces are needed
X_port = [x_np_r;   x_np_r  ];   % 6.4 → 3.0                              % 2-row x grid for port side
X_stbd = [x_np_fwd; x_np_fwd];   % 3.0 → 6.4                              % 2-row x grid for starboard side
Y_port = [y_np_r;   y_np_r  ];                                           % 2-row y grid for port side
Y_stbd = [y_np_fwd; y_np_fwd];                                           % 2-row y grid for starboard side
Z_g    = [z_np1 * ones(1,n_np); z_np0 * ones(1,n_np)];                   % 2-row z grid spanning plate height

% Port side  (Y > 0)                                                       % draw port texture-mapped plate
surf(X_port,  Y_port, Z_g, ...                                             % port plate surface geometry
     'CData', np_img, 'FaceColor', 'texturemap', ...                       % use image as texture
     'EdgeColor', [0.10 0.10 0.50], 'LineWidth', 1, ...                    % style plate border
     'FaceLighting', 'none');                                              % keep texture unaffected by lighting

% Starboard side  (Y < 0)  — X forward so text is not mirrored             % draw starboard texture-mapped plate
surf(X_stbd, -Y_stbd, Z_g, ...                                             % starboard plate surface geometry
     'CData', np_img, 'FaceColor', 'texturemap', ...                       % use image as texture
     'EdgeColor', [0.10 0.10 0.50], 'LineWidth', 1, ...                    % style plate border
     'FaceLighting', 'none');                                              % keep texture unaffected by lighting

% =========================================================                 % divider
% HULL TEXTURE MARKERS  (improve SfM feature detection)                    % add extra dots for features
% =========================================================                 % divider
% Grid of small dark-navy dots on both hull sides.                         % explain the markers
% Provides ~120 extra distinctive blob features for KAZE.                  % explain why useful
% Each dot is a tiny filled circle in the Y-Z plane.                       % explain geometry
% ---------------------------------------------------------                % divider
marker_col  = [0.12 0.12 0.45];   % dark navy                              % marker color
z_levels    = [zkeel+0.12, zkeel+0.27, zkeel+0.42, zkeel+0.56];            % z rows for markers
t_dot       = linspace(0, 2*pi, 10);                                       % circle samples for each dot
r_dot       = 0.016;                                                       % marker radius

for xi = 4 : 4 : length(x_ship) - 3                                        % loop through selected hull x positions
    xp = x_ship(xi);                                                       % x-position of current marker column
    yp = y_rail(xi);                                                       % hull half-width at that x
    if yp < 0.12, continue; end   % skip bow / stern taper                 % skip narrow tip regions

    for zp = z_levels                                                      % loop over marker height rows
        if zp > zdeck - 0.06, continue; end                                % skip markers too close to deck top

        % Port side dot  (Y > 0)                                           % describe port marker
        yd = yp * 1.004 + r_dot * 0.3 * cos(t_dot);  % flatten in Y        % build slightly flattened y circle
        zd = zp          + r_dot       * sin(t_dot);                        % build z circle
        fill3(xp * ones(1,10), yd, zd, marker_col, 'EdgeColor', 'none');   % draw port-side marker

        % Starboard mirror  (Y < 0)                                        % describe mirrored marker
        fill3(xp * ones(1,10), -yd, zd, marker_col, 'EdgeColor', 'none');  % draw starboard-side marker
    end
end

% =========================================================                 % divider
% HULL PANEL LINES  (horizontal plate seams)                               % add horizontal seam lines
% =========================================================                 % divider
% Thin contrasting lines across the hull: each line creates a              % explain why panel lines help
% row of distinct corner features where it crosses the dot grid.           % explain feature interaction
% ---------------------------------------------------------                % divider
panel_z   = [zkeel+0.10, zkeel+0.22, zkeel+0.34, zkeel+0.48, zkeel+0.60];  % z-heights of seam lines
panel_col = [0.38 0.28 0.18];   % slightly darker than hull brown          % seam color

for zpl = panel_z                                                          % loop over seam heights
    if zpl > zdeck - 0.04, continue; end                                   % skip line too close to deck top
    % Port side                                                            % describe port seam
    plot3(x_ship, y_rail * 1.002, zpl * ones(size(x_ship)), ...            % port seam coordinates
          '-', 'Color', panel_col, 'LineWidth', 0.8);                      % draw port seam
    % Starboard side                                                       % describe starboard seam
    plot3(x_ship, -y_rail * 1.002, zpl * ones(size(x_ship)), ...           % starboard seam coordinates
          '-', 'Color', panel_col, 'LineWidth', 0.8);                      % draw starboard seam
end

% =========================================================                 % divider
% APPEARANCE SETTINGS                                                      % final scene styling
% =========================================================                 % divider
colormap(vcol);                                                            % apply custom colormap from helper
axis([0 9 -1.5 1.5 0 3.5]);                                                % set scene display limits
axis equal;                                                                % enforce equal xyz scale
axis vis3d;                                                                % preserve 3D scaling during interaction
camproj('perspective');                                                    % use perspective camera
grid off;                                                                  % hide grid
axis off;                                                                  % hide axes
set(gcf, 'Color', 'w');                                                    % set figure background white
set(gca, 'Color', 'w');                                                    % set axes background white
lighting phong;                                                            % use Phong lighting
camlight headlight;                                                        % add headlight attached to viewer
title('');                                                                 % clear title
rotate3d on                                                                % allow interactive 3D rotation
view(3)                                                                    % set default 3D view