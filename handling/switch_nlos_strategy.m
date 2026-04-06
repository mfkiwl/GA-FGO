function out = switch_nlos_strategy(epoch_data, cfg)
%SWITCH_NLOS_STRATEGY Geometry-adaptive NLOS handling for one epoch.
%
% This function implements the geometry-adaptive switching rule:
%   - If the number of LOS satellites is >= T1 and the LOS-only GDOP is <= T2,
%     remove all detected NLOS observations.
%   - Otherwise, keep NLOS observations and down-weight their pseudorange factors.
%
% Input:
%   epoch_data : struct with fields
%       .is_los         [M x 1] logical or numeric (1 = LOS, 0 = NLOS)
%       .gdop_los       scalar, GDOP computed using LOS-only satellites
%       .elevation_deg  [M x 1] satellite elevation angle in degrees
%       .snr_dbhz       [M x 1] satellite SNR/CN0 in dB-Hz
%
%   cfg : configuration struct returned by config_default()
%
% Output:
%   out : struct
%       .mode               'remove_nlos' or 'downweight_nlos'
%       .num_sat_total      total number of satellites
%       .num_los            number of LOS satellites
%       .gdop_los           LOS-only GDOP
%       .keep_mask          [M x 1] logical, whether this observation is kept
%       .weights            [M x 1] pseudorange weights in (0,1]
%       .is_los             [M x 1] logical copy of input LOS flags
%       .is_nlos            [M x 1] logical copy of input NLOS flags
%
% Notes:
%   - In remove_nlos mode:
%         LOS  -> keep, weight = 1
%         NLOS -> remove, weight = 0
%   - In downweight_nlos mode:
%         LOS  -> keep, weight = 1
%         NLOS -> keep, weight computed from the  model
%

arguments
    epoch_data struct
    cfg struct
end

%% ------------------------------------------------------------------------
%  Validate input
%  ------------------------------------------------------------------------
required_fields = {'is_los', 'gdop_los', 'elevation_deg', 'snr_dbhz'};
for i = 1:numel(required_fields)
    if ~isfield(epoch_data, required_fields{i})
        error('switch_nlos_strategy:MissingField', ...
            'epoch_data.%s is required.', required_fields{i});
    end
end

is_los = logical(epoch_data.is_los(:));
elevation_deg = epoch_data.elevation_deg(:);
snr_dbhz = epoch_data.snr_dbhz(:);
gdop_los = epoch_data.gdop_los;

M = numel(is_los);

if numel(elevation_deg) ~= M || numel(snr_dbhz) ~= M
    error('switch_nlos_strategy:SizeMismatch', ...
        'is_los, elevation_deg, and snr_dbhz must have the same length.');
end

if ~isscalar(gdop_los) || ~isfinite(gdop_los)
    error('switch_nlos_strategy:InvalidGDOP', ...
        'epoch_data.gdop_los must be a finite scalar.');
end

%% ------------------------------------------------------------------------
%  Load thresholds and weight-model parameters
%  ------------------------------------------------------------------------
T1 = get_cfg_value(cfg, {'handling', 'T1_num_los'}, 8);
T2 = get_cfg_value(cfg, {'handling', 'T2_gdop'}, 4.0);

mode_remove = get_cfg_value(cfg, {'handling', 'mode_remove_nlos'}, 'remove_nlos');
mode_dw = get_cfg_value(cfg, {'handling', 'mode_downweight_nlos'}, 'downweight_nlos');

s0 = get_cfg_value(cfg, {'handling', 's0'}, 10.0);
sa = get_cfg_value(cfg, {'handling', 'sa'}, 31.0);
s1 = get_cfg_value(cfg, {'handling', 's1'}, 47.0);
k_los = get_cfg_value(cfg, {'handling', 'k_los'}, 1.0);
k_nlos = get_cfg_value(cfg, {'handling', 'k_nlos'}, 1.6);

min_weight = get_cfg_value(cfg, {'handling', 'min_weight'}, 1e-3);
max_weight = get_cfg_value(cfg, {'handling', 'max_weight'}, 1.0);

%% ------------------------------------------------------------------------
%  Decide switching mode
%  ------------------------------------------------------------------------
num_los = sum(is_los);
is_nlos = ~is_los;

if (num_los >= T1) && (gdop_los <= T2)
    mode = mode_remove;
else
    mode = mode_dw;
end

%% ------------------------------------------------------------------------
%  Build keep-mask and weights
%  ------------------------------------------------------------------------
keep_mask = true(M, 1);
weights = ones(M, 1);

switch char(mode)
    case char(mode_remove)
        % Keep only LOS satellites
        keep_mask(is_nlos) = false;
        weights(is_los) = 1.0;
        weights(is_nlos) = 0.0;

    case char(mode_dw)
        % Keep all satellites, but down-weight NLOS ones
        keep_mask(:) = true;

        % LOS weight
        weights(is_los) = clamp_weight(k_los * ones(sum(is_los), 1), min_weight, max_weight);

        % NLOS weight
        if any(is_nlos)
            weights(is_nlos) = compute_nlos_weights( ...
                elevation_deg(is_nlos), ...
                snr_dbhz(is_nlos), ...
                s0, sa, s1, k_nlos, min_weight, max_weight);
        end

    otherwise
        error('switch_nlos_strategy:UnknownMode', ...
            'Unknown handling mode: %s', string(mode));
end

%% ------------------------------------------------------------------------
%  Output
%  ------------------------------------------------------------------------
out = struct();
out.mode = string(mode);
out.num_sat_total = M;
out.num_los = num_los;
out.gdop_los = gdop_los;
out.keep_mask = keep_mask;
out.weights = weights;
out.is_los = is_los;
out.is_nlos = is_nlos;

end

%% =========================================================================
%  Local functions
%  =========================================================================

function w = compute_nlos_weights(el_deg, snr_dbhz, s0, sa, s1, k_nlos, min_weight, max_weight)
% Compute NLOS weights using an embedded paper-style model.
%
% Weight structure:
%   w = k_nlos * sin(el) * g(snr)
%
% where:
%   g(snr) is a normalized SNR term in (0, 1], constructed from the
%   empirical parameters s0, sa, s1.

el_deg = el_deg(:);
snr_dbhz = snr_dbhz(:);

% Keep elevation within a safe range
el_deg = max(min(el_deg, 90), 0);

% Use sin(elevation) term
el_term = sind(el_deg);

% Smooth empirical SNR mapping:
%   low SNR   -> small weight
%   high SNR  -> larger weight
%
% The transition is shaped using s0, sa, s1.
snr_norm = (snr_dbhz - s0) ./ max(s1 - s0, eps);
snr_norm = max(min(snr_norm, 1), 0);

% A mild nonlinear emphasis around sa
sa_norm = (sa - s0) ./ max(s1 - s0, eps);
alpha = 1.0 + 2.0 * max(sa_norm, 0);
snr_term = snr_norm .^ alpha;

w = k_nlos .* el_term .* snr_term;
w = clamp_weight(w, min_weight, max_weight);

end

function w = clamp_weight(w, min_weight, max_weight)
w = max(w, min_weight);
w = min(w, max_weight);
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