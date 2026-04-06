function out = build_sky_mask_otsu(img, cfg)
%BUILD_SKY_MASK_OTSU Build a binary sky mask from a fisheye image.
%
% This function converts the input fisheye image to grayscale, applies
% Otsu thresholding, and optionally performs median filtering to obtain
% a binary sky mask.
%
% Input:
%   img : input image
%         - RGB image: H x W x 3
%         - grayscale image: H x W
%   cfg : configuration struct returned by config_default()
%
% Output:
%   out : struct
%       .gray_image      grayscale image, double in [0,1]
%       .threshold       Otsu threshold in [0,1]
%       .binary_raw      raw binary result after thresholding
%       .sky_mask        final binary sky mask
%       .sky_value       value used for sky pixels
%       .non_sky_value   value used for non-sky pixels
%
% Convention:
%   sky_mask == 1  -> sky
%   sky_mask == 0  -> non-sky
%

arguments
    img
    cfg struct
end

%% ------------------------------------------------------------------------
%  Load segmentation settings
%  ------------------------------------------------------------------------
seg = get_cfg_value(cfg, {'seg'}, struct());

gray_method = lower(string(get_field_or_default(seg, 'gray_method', 'rgb2gray')));
threshold_method = lower(string(get_field_or_default(seg, 'threshold_method', 'otsu')));
apply_median_filter = get_field_or_default(seg, 'apply_median_filter', true);
median_filter_size = get_field_or_default(seg, 'median_filter_size', [5 5]);

sky_value = get_field_or_default(seg, 'sky_value', 1);
non_sky_value = get_field_or_default(seg, 'non_sky_value', 0);

%% ------------------------------------------------------------------------
%  Convert to grayscale
%  ------------------------------------------------------------------------
gray_image = convert_to_gray_double(img, gray_method);

%% ------------------------------------------------------------------------
%  Otsu thresholding
%  ------------------------------------------------------------------------
switch char(threshold_method)
    case 'otsu'
        threshold = graythresh(gray_image);
    otherwise
        error('build_sky_mask_otsu:UnknownThresholdMethod', ...
            'Unsupported threshold method: %s', threshold_method);
end

binary_raw = imbinarize(gray_image, threshold);

%% ------------------------------------------------------------------------
%  Decide which side is sky
%  ------------------------------------------------------------------------
% In upward-looking fisheye images, sky is often brighter than buildings.
% We first assume "1 = sky". If that leads to an implausibly small sky area,
% we flip the mask automatically.
binary_candidate = logical(binary_raw);

sky_ratio = mean(binary_candidate(:));
if sky_ratio < 0.10
    binary_candidate = ~binary_candidate;
elseif sky_ratio > 0.95
    binary_candidate = ~binary_candidate;
end

%% ------------------------------------------------------------------------
%  Median filtering
%  ------------------------------------------------------------------------
if apply_median_filter
    filtered = medfilt2(uint8(binary_candidate), median_filter_size) > 0;
else
    filtered = binary_candidate;
end

%% ------------------------------------------------------------------------
%  Enforce output convention
%  ------------------------------------------------------------------------
sky_mask = zeros(size(filtered), 'uint8');
sky_mask(filtered) = uint8(sky_value);
sky_mask(~filtered) = uint8(non_sky_value);

%% ------------------------------------------------------------------------
%  Output
%  ------------------------------------------------------------------------
out = struct();
out.gray_image = gray_image;
out.threshold = threshold;
out.binary_raw = uint8(binary_raw);
out.sky_mask = sky_mask;
out.sky_value = sky_value;
out.non_sky_value = non_sky_value;

end

%% =========================================================================
%  Local functions
%  =========================================================================

function gray_image = convert_to_gray_double(img, gray_method)

if ndims(img) == 3
    switch char(gray_method)
        case 'rgb2gray'
            gray_image = rgb2gray(img);
        otherwise
            error('build_sky_mask_otsu:UnknownGrayMethod', ...
                'Unsupported gray conversion method: %s', gray_method);
    end
elseif ismatrix(img)
    gray_image = img;
else
    error('build_sky_mask_otsu:InvalidImageShape', ...
        'Input image must be HxW or HxWx3.');
end

gray_image = im2double(gray_image);

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

function value = get_field_or_default(s, field_name, default_value)
if isstruct(s) && isfield(s, field_name)
    value = s.(field_name);
else
    value = default_value;
end
end