function proj = project_satellite_to_image(azimuth_deg, elevation_deg, heading_deg, cfg)
%PROJECT_SATELLITE_TO_IMAGE Project satellite directions onto a fisheye image.
%
% This function implements the fisheye projection used in the manuscript:
%
%   theta_img = theta_sat_az + theta_heading
%   gamma     = pi/2 - theta_sat_el
%   r         = 2 * f * tan(gamma / 2)
%   x         = cx + r * cos(theta_img)
%   y         = cy - r * sin(theta_img)
%
% Input:
%   azimuth_deg   : [N x 1] satellite azimuth angles in degrees
%   elevation_deg : [N x 1] satellite elevation angles in degrees
%   heading_deg   : scalar or [N x 1], camera/image heading in degrees
%   cfg           : configuration struct from config_default()
%
% Output:
%   proj : struct
%       .x_pixel         [N x 1] projected x coordinates in image
%       .y_pixel         [N x 1] projected y coordinates in image
%       .r_pixel         [N x 1] radial distances in pixels
%       .gamma_deg       [N x 1] incident angles in degrees
%       .theta_img_deg   [N x 1] heading-corrected azimuths in degrees
%       .valid_mask      [N x 1] projection inside image bounds
%       .inside_image    [N x 1] same as valid_mask
%
% Notes:
%   - Image x increases to the right.
%   - Image y increases downward.
%   - The current implementation uses the stereographic fisheye model:
%         r = 2*f*tan(gamma/2)
%

arguments
    azimuth_deg (:,1) double
    elevation_deg (:,1) double
    heading_deg double
    cfg struct
end

%% ------------------------------------------------------------------------
%  Validate input size
%  ------------------------------------------------------------------------
N = numel(azimuth_deg);

if numel(elevation_deg) ~= N
    error('project_satellite_to_image:SizeMismatch', ...
        'azimuth_deg and elevation_deg must have the same length.');
end

if isscalar(heading_deg)
    heading_deg = repmat(heading_deg, N, 1);
elseif numel(heading_deg) ~= N
    error('project_satellite_to_image:InvalidHeadingSize', ...
        'heading_deg must be either a scalar or an N-by-1 vector.');
else
    heading_deg = heading_deg(:);
end

%% ------------------------------------------------------------------------
%  Load camera parameters
%  ------------------------------------------------------------------------
camera = get_cfg_value(cfg, {'camera'}, struct());

f  = get_field_or_default(camera, 'f', 520.0);
cx = get_field_or_default(camera, 'cx', 960.0);
cy = get_field_or_default(camera, 'cy', 540.0);

img_w = get_field_or_default(camera, 'image_width', 1920);
img_h = get_field_or_default(camera, 'image_height', 1080);

model = lower(string(get_field_or_default(camera, 'model', 'stereographic_fisheye')));

%% ------------------------------------------------------------------------
%  Normalize angles
%  ------------------------------------------------------------------------
% Keep azimuth in [0, 360)
azimuth_deg = wrap_to_360_local(azimuth_deg);

% Clamp elevation to [0, 90]
elevation_deg = max(min(elevation_deg, 90), 0);

% Heading-corrected azimuth
theta_img_deg = azimuth_deg + heading_deg;
theta_img_deg = wrap_to_360_local(theta_img_deg);

% Incident angle gamma = 90 deg - elevation
gamma_deg = 90 - elevation_deg;
gamma_rad = deg2rad(gamma_deg);

theta_img_rad = deg2rad(theta_img_deg);

%% ------------------------------------------------------------------------
%  Fisheye radial mapping
%  ------------------------------------------------------------------------
switch char(model)
    case 'stereographic_fisheye'
        % r = 2 * f * tan(gamma / 2)
        r_pixel = 2 .* f .* tan(gamma_rad ./ 2);

    case 'equidistant_fisheye'
        % Optional fallback:
        % r = f * gamma
        r_pixel = f .* gamma_rad;

    otherwise
        error('project_satellite_to_image:UnknownCameraModel', ...
            'Unsupported camera model: %s', model);
end

%% ------------------------------------------------------------------------
%  Image-plane projection
%  ------------------------------------------------------------------------
x_pixel = cx + r_pixel .* cos(theta_img_rad);
y_pixel = cy - r_pixel .* sin(theta_img_rad);

%% ------------------------------------------------------------------------
%  Check image bounds
%  ------------------------------------------------------------------------
inside_x = (x_pixel >= 1) & (x_pixel <= img_w);
inside_y = (y_pixel >= 1) & (y_pixel <= img_h);
inside_image = inside_x & inside_y;

%% ------------------------------------------------------------------------
%  Output
%  ------------------------------------------------------------------------
proj = struct();
proj.x_pixel = x_pixel;
proj.y_pixel = y_pixel;
proj.r_pixel = r_pixel;
proj.gamma_deg = gamma_deg;
proj.theta_img_deg = theta_img_deg;
proj.valid_mask = inside_image;
proj.inside_image = inside_image;

end

%% =========================================================================
%  Local functions
%  =========================================================================

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

function value = get_field_or_default(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name)
    value = s.(field_name);
else
    value = default_value;
end
end

function angle_deg = wrap_to_360_local(angle_deg)
angle_deg = mod(angle_deg, 360);
angle_deg(angle_deg < 0) = angle_deg(angle_deg < 0) + 360;
end