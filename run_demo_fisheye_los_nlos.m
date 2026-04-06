function demo = run_demo_fisheye_los_nlos()
%RUN_DEMO_FISHEYE_LOS_NLOS Demo for fisheye-image-based LOS/NLOS classification.
%
% This demo performs:
%   1) load configuration
%   2) read one fisheye image
%   3) build sky mask using Otsu thresholding
%   4) project demo satellites to the image plane
%   5) classify LOS/NLOS by querying the sky mask
%   6) visualize the result
%
% Required files in the minimal repository:
%   - config/config_default.m
%   - preprocess/build_sky_mask_otsu.m
%   - projection/project_satellite_to_image.m
%
% Output:
%   demo : struct containing the demo inputs and outputs

clc;

%% ------------------------------------------------------------------------
%  Load configuration
%  ------------------------------------------------------------------------
cfg = config_default();

%% ------------------------------------------------------------------------
%  Read fisheye image
%  ------------------------------------------------------------------------
img_path = cfg.data.fisheye_image;

if ~exist(img_path, 'file')
    error('run_demo_fisheye_los_nlos:MissingImage', ...
        'Demo fisheye image not found: %s', img_path);
end

img = imread(img_path);

%% ------------------------------------------------------------------------
%  Build sky mask
%  ------------------------------------------------------------------------
mask_out = build_sky_mask_otsu(img, cfg);
sky_mask = logical(mask_out.sky_mask);

%% ------------------------------------------------------------------------
%  Load demo heading
%  ------------------------------------------------------------------------
heading_deg = try_load_heading(cfg);

%% ------------------------------------------------------------------------
%  Load demo satellite azimuth/elevation
%  ------------------------------------------------------------------------
[azimuth_deg, elevation_deg, sat_name] = try_load_satellite_demo(cfg);

%% ------------------------------------------------------------------------
%  Project satellites to fisheye image
%  ------------------------------------------------------------------------
proj = project_satellite_to_image(azimuth_deg, elevation_deg, heading_deg, cfg);

%% ------------------------------------------------------------------------
%  Classify LOS/NLOS from sky mask
%  ------------------------------------------------------------------------
[is_los, is_inside, pixel_xy] = classify_from_mask(sky_mask, proj, cfg);

%% ------------------------------------------------------------------------
%  Prepare output
%  ------------------------------------------------------------------------
demo = struct();
demo.cfg = cfg;
demo.image_path = img_path;
demo.heading_deg = heading_deg;
demo.azimuth_deg = azimuth_deg;
demo.elevation_deg = elevation_deg;
demo.sat_name = sat_name;
demo.gray_image = mask_out.gray_image;
demo.threshold = mask_out.threshold;
demo.sky_mask = sky_mask;
demo.proj = proj;
demo.pixel_xy = pixel_xy;
demo.is_inside = is_inside;
demo.is_los = is_los;
demo.is_nlos = ~is_los;

%% ------------------------------------------------------------------------
%  Print summary
%  ------------------------------------------------------------------------
num_total = numel(is_los);
num_los = sum(is_los);
num_nlos = sum(~is_los);

fprintf('\n');
fprintf('=== Fisheye LOS/NLOS Demo Summary ===\n');
fprintf('Image path   : %s\n', img_path);
fprintf('Heading (deg): %.2f\n', heading_deg);
fprintf('Threshold    : %.4f\n', mask_out.threshold);
fprintf('Total sats   : %d\n', num_total);
fprintf('LOS sats     : %d\n', num_los);
fprintf('NLOS sats    : %d\n', num_nlos);
fprintf('\n');

for i = 1:num_total
    status_str = 'NLOS';
    if is_los(i)
        status_str = 'LOS';
    end
    fprintf('%-6s  az = %7.2f deg, el = %6.2f deg, x = %8.2f, y = %8.2f, %s\n', ...
        sat_name{i}, azimuth_deg(i), elevation_deg(i), ...
        proj.x_pixel(i), proj.y_pixel(i), status_str);
end

%% ------------------------------------------------------------------------
%  Visualization
%  ------------------------------------------------------------------------
if isfield(cfg, 'output') && isfield(cfg.output, 'show_figures') && cfg.output.show_figures
    plot_demo_results(img, sky_mask, proj, is_los, sat_name);
end

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
        name_i = candidate_names{i};
        if any(strcmpi(T.Properties.VariableNames, name_i))
            heading_deg = T.(T.Properties.VariableNames{strcmpi(T.Properties.VariableNames, name_i)});
            heading_deg = heading_deg(1);
            return;
        end
    end

    % fallback: use the first numeric value in the first row
    row1 = T(1,:);
    for j = 1:width(row1)
        val = row1{1,j};
        if isnumeric(val)
            heading_deg = val(1);
            return;
        end
    end
catch
    warning('run_demo_fisheye_los_nlos:HeadingLoadFailed', ...
        'Failed to load heading file. heading_deg = 0 will be used.');
end

end

function [azimuth_deg, elevation_deg, sat_name] = try_load_satellite_demo(cfg)

azimuth_deg = [];
elevation_deg = [];
sat_name = {};

if isfield(cfg, 'data') && isfield(cfg.data, 'sat_info_file')
    sat_file = cfg.data.sat_info_file;
else
    sat_file = '';
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
                raw_id = T.(id_col);
                sat_name = convert_ids_to_cellstr(raw_id);
            else
                sat_name = default_sat_names(numel(azimuth_deg));
            end
        end
    catch
        warning('run_demo_fisheye_los_nlos:SatelliteLoadFailed', ...
            'Failed to load satellite demo file. Built-in demo satellites will be used.');
    end
end

% Fallback demo satellites
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
    n = numel(raw_id);
    cellstr_ids = default_sat_names(n);
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

function plot_demo_results(img, sky_mask, proj, is_los, sat_name)

figure('Name', 'Fisheye LOS/NLOS Demo - Original Image');
imshow(img);
title('Original Fisheye Image');

figure('Name', 'Fisheye LOS/NLOS Demo - Sky Mask');
imshow(sky_mask, []);
title('Sky Mask');

figure('Name', 'Fisheye LOS/NLOS Demo - Projection and Classification');
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

    plot(proj.x_pixel(i), proj.y_pixel(i), mk, ...
        'MarkerSize', 8, 'LineWidth', 1.5);

    text(proj.x_pixel(i) + 8, proj.y_pixel(i), sat_name{i}, ...
        'Color', txt_color, 'FontSize', 9, 'FontWeight', 'bold');
end

title('Projected Satellites and LOS/NLOS Classification');
legend({'LOS','NLOS'}, 'Location', 'best');
hold off;

end