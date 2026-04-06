function demo = run_demo_ga_fgo()
%RUN_DEMO_GA_FGO Minimal end-to-end demo of the GA-FGO pipeline.
%
% This demo performs:
%   1) load configuration
%   2) load one fisheye image and build sky mask
%   3) project demo satellites onto the image plane
%   4) classify LOS/NLOS from the sky mask
%   5) compute a simple LOS-only GDOP indicator
%   6) apply geometry-adaptive NLOS handling
%   7) build a small sliding-window pseudorange/TDCP demo dataset
%   8) solve the window using LM-based GA-FGO
%
% Required files:
%   - config/config_default.m
%   - preprocess/build_sky_mask_otsu.m
%   - projection/project_satellite_to_image.m
%   - handling/switch_nlos_strategy.m
%   - fgo/solve_ga_fgo_lm.m
%
% Output:
%   demo : struct containing intermediate and final results

clc;

%% ------------------------------------------------------------------------
%  Load configuration
%  ------------------------------------------------------------------------
cfg = config_default();

fprintf('\n');
fprintf('=== GA-FGO Minimal Demo ===\n');

%% ------------------------------------------------------------------------
%  Read fisheye image and build sky mask
%  ------------------------------------------------------------------------
img_path = cfg.data.fisheye_image;
if ~exist(img_path, 'file')
    error('run_demo_ga_fgo:MissingImage', ...
        'Demo fisheye image not found: %s', img_path);
end

img = imread(img_path);
mask_out = build_sky_mask_otsu(img, cfg);
sky_mask = logical(mask_out.sky_mask);

%% ------------------------------------------------------------------------
%  Load heading and demo satellites
%  ------------------------------------------------------------------------
heading_deg = try_load_heading(cfg);
[azimuth_deg, elevation_deg, sat_name] = try_load_satellite_demo(cfg);

%% ------------------------------------------------------------------------
%  Project satellites to image and classify LOS/NLOS
%  ------------------------------------------------------------------------
proj = project_satellite_to_image(azimuth_deg, elevation_deg, heading_deg, cfg);
[is_los, is_inside, pixel_xy] = classify_from_mask(sky_mask, proj, cfg);

%% ------------------------------------------------------------------------
%  Compute LOS-only GDOP approximation
%  ------------------------------------------------------------------------
gdop_los = approximate_gdop_from_azel( ...
    azimuth_deg(is_los), elevation_deg(is_los));

epoch_data = struct();
epoch_data.is_los = is_los;
epoch_data.gdop_los = gdop_los;
epoch_data.elevation_deg = elevation_deg;
epoch_data.snr_dbhz = build_demo_snr(is_los, elevation_deg);

handle_out = switch_nlos_strategy(epoch_data, cfg);

fprintf('Mode selected : %s\n', handle_out.mode);
fprintf('Total sats    : %d\n', handle_out.num_sat_total);
fprintf('LOS sats      : %d\n', handle_out.num_los);
fprintf('LOS-only GDOP : %.4f\n', handle_out.gdop_los);

%% ------------------------------------------------------------------------
%  Build demo sliding-window observations
%  ------------------------------------------------------------------------
demo_data = build_demo_window_observations( ...
    azimuth_deg, elevation_deg, sat_name, is_los, handle_out, cfg);

%% ------------------------------------------------------------------------
%  Solve GA-FGO over the active window
%  ------------------------------------------------------------------------
result = solve_ga_fgo_lm(demo_data, cfg);

%% ------------------------------------------------------------------------
%  Print result summary
%  ------------------------------------------------------------------------
pos_err = vecnorm(result.position_ecef - demo_data.true_position_ecef, 2, 2);

fprintf('\n');
fprintf('=== Optimization Summary ===\n');
fprintf('Converged     : %d\n', result.converged);
fprintf('Iterations    : %d\n', result.num_iterations);
fprintf('Final cost    : %.6e\n', result.final_cost);
fprintf('Mean 3D error : %.4f m\n', mean(pos_err));
fprintf('Max  3D error : %.4f m\n', max(pos_err));

%% ------------------------------------------------------------------------
%  Visualization
%  ------------------------------------------------------------------------
if isfield(cfg, 'output') && isfield(cfg.output, 'show_figures') && cfg.output.show_figures
    plot_demo_overview(img, sky_mask, proj, is_los, sat_name, ...
        handle_out, demo_data, result);
end

%% ------------------------------------------------------------------------
%  Pack output
%  ------------------------------------------------------------------------
demo = struct();
demo.cfg = cfg;
demo.image_path = img_path;
demo.gray_image = mask_out.gray_image;
demo.threshold = mask_out.threshold;
demo.sky_mask = sky_mask;
demo.heading_deg = heading_deg;
demo.azimuth_deg = azimuth_deg;
demo.elevation_deg = elevation_deg;
demo.sat_name = sat_name;
demo.proj = proj;
demo.pixel_xy = pixel_xy;
demo.is_inside = is_inside;
demo.is_los = is_los;
demo.is_nlos = ~is_los;
demo.gdop_los = gdop_los;
demo.handling = handle_out;
demo.demo_data = demo_data;
demo.result = result;
demo.position_error_3d = pos_err;

end

%% =========================================================================
%  Local functions
%  =========================================================================

function heading_deg = try_load_heading(cfg)

heading_deg = 0.0;

if ~isfield(cfg, 'data') || ~isfield(cfg.data, 'heading_file')
    return;
end

heading_file = cfg.data.heading_file;

if ~exist(heading_file, 'file')
    return;
end

try
    T = readtable(heading_file);

    candidate_names = {'heading_deg', 'heading', 'yaw_deg', 'yaw'};
    for i = 1:numel(candidate_names)
        idx = find(strcmpi(T.Properties.VariableNames, candidate_names{i}), 1);
        if ~isempty(idx)
            heading_deg = T.(T.Properties.VariableNames{idx});
            heading_deg = heading_deg(1);
            return;
        end
    end
catch
    warning('run_demo_ga_fgo:HeadingLoadFailed', ...
        'Failed to load heading file. heading_deg = 0 will be used.');
end

end

function [azimuth_deg, elevation_deg, sat_name] = try_load_satellite_demo(cfg)

azimuth_deg = [];
elevation_deg = [];
sat_name = {};

sat_file = '';
if isfield(cfg, 'data') && isfield(cfg.data, 'sat_info_file')
    sat_file = cfg.data.sat_info_file;
end

if ~isempty(sat_file) && exist(sat_file, 'file')
    try
        T = readtable(sat_file);

        az_col = find_matching_column(T.Properties.VariableNames, ...
            {'azimuth_deg','az_deg','azimuth','az'});
        el_col = find_matching_column(T.Properties.VariableNames, ...
            {'elevation_deg','el_deg','elevation','el'});
        id_col = find_matching_column(T.Properties.VariableNames, ...
            {'sat_name','satellite','prn','sat_id','sv'});

        if ~isempty(az_col) && ~isempty(el_col)
            azimuth_deg = T.(az_col);
            elevation_deg = T.(el_col);

            if ~isempty(id_col)
                sat_name = convert_ids_to_cellstr(T.(id_col));
            else
                sat_name = default_sat_names(numel(azimuth_deg));
            end
        end
    catch
        warning('run_demo_ga_fgo:SatelliteLoadFailed', ...
            'Failed to load satellite demo file. Built-in demo satellites will be used.');
    end
end

if isempty(azimuth_deg) || isempty(elevation_deg)
    azimuth_deg = [15; 48; 82; 125; 163; 210; 248; 286; 322; 350];
    elevation_deg = [72; 58; 35; 41; 23; 66; 29; 18; 52; 11];
    sat_name = {'G01','G03','G08','C12','E05','G15','C19','E11','G24','C30'};
end

azimuth_deg = azimuth_deg(:);
elevation_deg = elevation_deg(:);

if isempty(sat_name)
    sat_name = default_sat_names(numel(azimuth_deg));
end

end

function name = find_matching_column(var_names, candidates)

name = '';
for i = 1:numel(candidates)
    idx = find(strcmpi(var_names, candidates{i}), 1);
    if ~isempty(idx)
        name = var_names{idx};
        return;
    end
end

end

function cellstr_ids = convert_ids_to_cellstr(raw_id)

if iscell(raw_id)
    cellstr_ids = cellfun(@string, raw_id, 'UniformOutput', false);
    cellstr_ids = cellfun(@char, cellstr_ids, 'UniformOutput', false);
elseif isstring(raw_id)
    cellstr_ids = cellstr(raw_id);
elseif isnumeric(raw_id)
    cellstr_ids = arrayfun(@(x) sprintf('SAT%02d', x), raw_id, 'UniformOutput', false);
else
    cellstr_ids = default_sat_names(numel(raw_id));
end

end

function sat_name = default_sat_names(n)
sat_name = arrayfun(@(k) sprintf('SAT%02d', k), 1:n, 'UniformOutput', false);
end

function [is_los, is_inside, pixel_xy] = classify_from_mask(sky_mask, proj, cfg)

round_method = 'nearest';
if isfield(cfg, 'classifier') && isfield(cfg.classifier, 'round_pixel_method')
    round_method = lower(string(cfg.classifier.round_pixel_method));
end

H = size(sky_mask, 1);
W = size(sky_mask, 2);
N = numel(proj.x_pixel);

x = proj.x_pixel(:);
y = proj.y_pixel(:);

switch char(round_method)
    case 'nearest'
        xi = round(x);
        yi = round(y);
    case 'floor'
        xi = floor(x);
        yi = floor(y);
    case 'ceil'
        xi = ceil(x);
        yi = ceil(y);
    otherwise
        xi = round(x);
        yi = round(y);
end

is_inside = (xi >= 1) & (xi <= W) & (yi >= 1) & (yi <= H);
is_los = false(N, 1);

for i = 1:N
    if is_inside(i)
        is_los(i) = sky_mask(yi(i), xi(i)) > 0;
    else
        is_los(i) = false;
    end
end

pixel_xy = [xi, yi];

end

function snr_dbhz = build_demo_snr(is_los, elevation_deg)

% Simple demo SNR model
snr_dbhz = 22 + 0.22 .* elevation_deg(:);
snr_dbhz(is_los) = snr_dbhz(is_los) + 8.0;
snr_dbhz(~is_los) = snr_dbhz(~is_los) - 3.0;
snr_dbhz = max(min(snr_dbhz, 48), 15);

end

function gdop = approximate_gdop_from_azel(az_deg, el_deg)

% A compact GDOP approximation from LOS satellite geometry for demo use.
%
% Build a standard geometry matrix:
%   G_i = [ -u_x  -u_y  -u_z  1 ]
% where u is the line-of-sight unit vector in ENU.

if numel(az_deg) < 4
    gdop = inf;
    return;
end

az_deg = az_deg(:);
el_deg = el_deg(:);

u_e = cosd(el_deg) .* sind(az_deg);
u_n = cosd(el_deg) .* cosd(az_deg);
u_u = sind(el_deg);

G = [-u_e, -u_n, -u_u, ones(numel(az_deg), 1)];

Nmat = G' * G;
if rcond(Nmat) < 1e-12
    gdop = inf;
    return;
end

Q = inv(Nmat);
gdop = sqrt(trace(Q));

end

function data = build_demo_window_observations( ...
    azimuth_deg, elevation_deg, sat_name, is_los, handle_out, cfg)

N = min(get_cfg_value(cfg, {'fgo', 'window_length'}, 15), 15);
M = numel(azimuth_deg);

% Wuhan-like demo origin
lat0_deg = 30.54;
lon0_deg = 114.36;
h0_m = 20.0;

p0 = llh_to_ecef(lat0_deg, lon0_deg, h0_m);

% Simple linear motion in ENU
enu_step = [1.2, 0.4, 0.0]; % m/epoch
true_position_ecef = zeros(N, 3);
true_clock_bias_m = zeros(N, 1);

for k = 1:N
    enu_k = (k - 1) .* enu_step;
    true_position_ecef(k, :) = (p0 + enu_to_ecef_delta(enu_k(:), lat0_deg, lon0_deg))';
    true_clock_bias_m(k) = 3.0 + 0.05 * (k - 1);
end

% Initial states for optimization
initial_pos_ecef = true_position_ecef + 2.5 .* randn(N, 3);
initial_clk_bias_m = true_clock_bias_m + 2.0 .* randn(N, 1);

% Fixed satellite positions synthesized from az/el around the origin
slant_range_m = 2.02e7;
sat_pos_ecef = zeros(M, 3);
for j = 1:M
    los_enu = azel_to_enu_unit(azimuth_deg(j), elevation_deg(j));
    sat_pos_ecef(j, :) = (p0 + slant_range_m .* enu_to_ecef_delta(los_enu, lat0_deg, lon0_deg))';
end

% Build pseudorange observations
pr_obs = cell(N, 1);

base_sigma_pr = get_cfg_value(cfg, {'fgo', 'sigma_pr'}, 5.0);
min_w = get_cfg_value(cfg, {'handling', 'min_weight'}, 1e-3);

for k = 1:N
    sat_pos_k = [];
    rho_k = [];
    sigma_k = [];
    sat_name_k = {};
    is_los_k = [];
    weight_k = [];

    for j = 1:M
        if ~handle_out.keep_mask(j)
            continue;
        end

        p = true_position_ecef(k, :)';
        s = sat_pos_ecef(j, :)';
        rho_true = norm(p - s) + true_clock_bias_m(k);

        % Add NLOS bias only for pseudorange
        nlos_bias = 0.0;
        if ~is_los(j)
            nlos_bias = 12.0 + 6.0 * rand();
        end

        sigma_eff = base_sigma_pr / sqrt(max(handle_out.weights(j), min_w));
        rho_meas = rho_true + nlos_bias + sigma_eff * randn();

        sat_pos_k = [sat_pos_k; s'];
        rho_k = [rho_k; rho_meas];
        sigma_k = [sigma_k; sigma_eff];
        sat_name_k{end+1,1} = sat_name{j}; %#ok<AGROW>
        is_los_k = [is_los_k; is_los(j)];
        weight_k = [weight_k; handle_out.weights(j)];
    end

    obs_k = struct();
    obs_k.sat_pos_ecef = sat_pos_k;
    obs_k.rho_m = rho_k;
    obs_k.sigma_m = sigma_k;
    obs_k.sat_name = sat_name_k;
    obs_k.is_los = is_los_k;
    obs_k.weight = weight_k;

    pr_obs{k} = obs_k;
end

% Build TDCP observations
tdcp_obs = cell(N - 1, 1);
base_sigma_tdcp = get_cfg_value(cfg, {'fgo', 'sigma_tdcp'}, 0.05);

for k = 1:(N - 1)
    sat_prev = [];
    sat_curr = [];
    tdcp_m = [];
    sigma_m = [];
    sat_name_k = {};

    for j = 1:M
        if ~handle_out.keep_mask(j)
            continue;
        end

        p1 = true_position_ecef(k, :)';
        p2 = true_position_ecef(k + 1, :)';
        s1 = sat_pos_ecef(j, :)';
        s2 = sat_pos_ecef(j, :)';

        tdcp_true = (norm(p2 - s2) - norm(p1 - s1)) + ...
                    (true_clock_bias_m(k + 1) - true_clock_bias_m(k));

        tdcp_meas = tdcp_true + base_sigma_tdcp * randn();

        sat_prev = [sat_prev; s1'];
        sat_curr = [sat_curr; s2'];
        tdcp_m = [tdcp_m; tdcp_meas];
        sigma_m = [sigma_m; base_sigma_tdcp];
        sat_name_k{end+1,1} = sat_name{j}; %#ok<AGROW>
    end

    obs_k = struct();
    obs_k.sat_pos_prev_ecef = sat_prev;
    obs_k.sat_pos_curr_ecef = sat_curr;
    obs_k.tdcp_m = tdcp_m;
    obs_k.sigma_m = sigma_m;
    obs_k.sat_name = sat_name_k;

    tdcp_obs{k} = obs_k;
end

data = struct();
data.initial_pos_ecef = initial_pos_ecef;
data.initial_clk_bias_m = initial_clk_bias_m;
data.pr_obs = pr_obs;
data.tdcp_obs = tdcp_obs;

data.true_position_ecef = true_position_ecef;
data.true_clock_bias_m = true_clock_bias_m;
data.sat_pos_ecef = sat_pos_ecef;
data.sat_name = sat_name;
data.is_los = is_los;
data.handle_out = handle_out;

end

function value = get_cfg_value(cfg, field_path, default_value)

value = default_value;
tmp = cfg;

for i = 1:numel(field_path)
    key = field_path{i};
    if isstruct(tmp) && isfield(tmp, key)
        tmp = tmp.(key);
    else
        return;
    end
end

value = tmp;

end

function ecef = llh_to_ecef(lat_deg, lon_deg, h_m)

a = 6378137.0;
f = 1 / 298.257223563;
e2 = f * (2 - f);

lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);

N = a / sqrt(1 - e2 * sin(lat)^2);

x = (N + h_m) * cos(lat) * cos(lon);
y = (N + h_m) * cos(lat) * sin(lon);
z = (N * (1 - e2) + h_m) * sin(lat);

ecef = [x; y; z];

end

function delta_ecef = enu_to_ecef_delta(enu_vec, lat_deg, lon_deg)

lat = deg2rad(lat_deg);
lon = deg2rad(lon_deg);

R = [ ...
    -sin(lon), -sin(lat)*cos(lon),  cos(lat)*cos(lon); ...
     cos(lon), -sin(lat)*sin(lon),  cos(lat)*sin(lon); ...
          0  ,       cos(lat)     ,       sin(lat)     ];

delta_ecef = R * enu_vec(:);

end

function u_enu = azel_to_enu_unit(az_deg, el_deg)

u_e = cosd(el_deg) * sind(az_deg);
u_n = cosd(el_deg) * cosd(az_deg);
u_u = sind(el_deg);

u_enu = [u_e; u_n; u_u];
u_enu = u_enu / norm(u_enu);

end

function plot_demo_overview(img, sky_mask, proj, is_los, sat_name, ...
    handle_out, demo_data, result)

figure('Name', 'GA-FGO Demo - Image and Classification');
imshow(img);
hold on;
for i = 1:numel(is_los)
    if is_los(i)
        mk = 'go';
        txt_color = 'g';
    else
        mk = 'ro';
        txt_color = 'r';
    end

    if ~handle_out.keep_mask(i)
        mk = 'ko';
        txt_color = 'k';
    end

    plot(proj.x_pixel(i), proj.y_pixel(i), mk, ...
        'MarkerSize', 8, 'LineWidth', 1.5);

    text(proj.x_pixel(i) + 8, proj.y_pixel(i), sat_name{i}, ...
        'Color', txt_color, 'FontSize', 9, 'FontWeight', 'bold');
end

hold off;

figure('Name', 'GA-FGO Demo - Sky Mask');
imshow(sky_mask, []);


figure('Name', 'GA-FGO Demo - Position Error');
pos_err = vecnorm(result.position_ecef - demo_data.true_position_ecef, 2, 2);
plot(pos_err, 'LineWidth', 1.5);
grid on;
xlabel('Epoch');
ylabel('3D Position Error (m)');


figure('Name', 'GA-FGO Demo - Clock Bias');
plot(demo_data.true_clock_bias_m, 'k-', 'LineWidth', 1.5); hold on;
plot(result.clock_bias_m, 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Epoch');
ylabel('Clock Bias (m)');
legend('True', 'Estimated', 'Location', 'best');


end