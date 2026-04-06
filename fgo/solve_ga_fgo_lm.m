function result = solve_ga_fgo_lm(data, cfg)
%SOLVE_GA_FGO_LM Solve one active window using LM optimization.
%
% State definition for epoch k:
%   x_k = [p_k(3); cb_k(1)]
% where:
%   p_k  : receiver position in ECEF (m)
%   cb_k : receiver clock bias in meters
%
% This function solves a single active window with:
%   1) pseudorange factors
%   2) TDCP factors
%   3) prior factor on the first state
%
% Expected input structure "data":
%   data.initial_pos_ecef   : [N x 3]
%   data.initial_clk_bias_m : [N x 1] (optional, default zeros)
%   data.pr_obs             : {N x 1} cell
%       data.pr_obs{k}.sat_pos_ecef : [Mk x 3]
%       data.pr_obs{k}.rho_m        : [Mk x 1]
%       data.pr_obs{k}.sigma_m      : [Mk x 1] (optional)
%
%   data.tdcp_obs           : {N-1 x 1} cell (optional)
%       data.tdcp_obs{k}.sat_pos_prev_ecef : [Lk x 3]
%       data.tdcp_obs{k}.sat_pos_curr_ecef : [Lk x 3]
%       data.tdcp_obs{k}.tdcp_m            : [Lk x 1]
%       data.tdcp_obs{k}.sigma_m           : [Lk x 1] (optional)
%
% Output "result":
%   result.position_ecef    : [N x 3]
%   result.clock_bias_m     : [N x 1]
%   result.num_epochs       : scalar
%   result.num_iterations   : scalar
%   result.converged        : logical
%   result.final_cost       : scalar
%   result.cost_history     : [iter x 1]
%   result.state_vector     : [4N x 1]

arguments
    data struct
    cfg struct
end

%% ------------------------------------------------------------------------
%  Validate input
%  ------------------------------------------------------------------------
validate_input_data(data, cfg);

N = size(data.initial_pos_ecef, 1);

if isfield(data, 'initial_clk_bias_m') && ~isempty(data.initial_clk_bias_m)
    initial_clk = data.initial_clk_bias_m(:);
else
    initial_clk = zeros(N, 1);
end

%% ------------------------------------------------------------------------
%  Build initial state vector
%  ------------------------------------------------------------------------
x = zeros(4 * N, 1);
for k = 1:N
    idx = state_index(k);
    x(idx.pos) = data.initial_pos_ecef(k, :)';
    x(idx.cb)  = initial_clk(k);
end

x0 = x;

%% ------------------------------------------------------------------------
%  Solver settings
%  ------------------------------------------------------------------------
max_iter = get_cfg_value(cfg, {'fgo', 'max_iterations'}, 100);
rel_tol  = get_cfg_value(cfg, {'fgo', 'relative_error_tol'}, 1e-6);
abs_tol  = get_cfg_value(cfg, {'fgo', 'absolute_error_tol'}, 1e-6);
verbose  = get_cfg_value(cfg, {'output', 'verbose'}, true);

lambda = 1e-3;
lambda_min = 1e-8;
lambda_max = 1e8;

cost_history = zeros(max_iter, 1);
converged = false;

%% ------------------------------------------------------------------------
%  LM optimization
%  ------------------------------------------------------------------------
for iter = 1:max_iter
    [r, J, w, meta] = build_system(x, x0, data, cfg);
    cost = compute_cost(r, w);

    cost_history(iter) = cost;

    H = J' * spdiags(w, 0, numel(w), numel(w)) * J;
    g = J' * (w .* r);

    H_lm = H + lambda * speye(size(H, 1));

    dx = -H_lm \ g;

    if any(~isfinite(dx))
        warning('solve_ga_fgo_lm:NumericalIssue', ...
            'Non-finite increment detected. Optimization stopped.');
        break;
    end

    x_trial = x + dx;
    [r_trial, ~, w_trial] = build_system(x_trial, x0, data, cfg);
    cost_trial = compute_cost(r_trial, w_trial);

    if verbose
        fprintf('[solve_ga_fgo_lm] Iter %03d | cost = %.6e | lambda = %.3e | pr = %d | tdcp = %d\n', ...
            iter, cost, lambda, meta.num_pr_factors, meta.num_tdcp_factors);
    end

    if cost_trial < cost
        x = x_trial;
        lambda = max(lambda / 10, lambda_min);

        step_norm = norm(dx);
        cost_change = abs(cost - cost_trial);

        if step_norm < abs_tol || cost_change < rel_tol * max(1.0, cost)
            converged = true;
            cost_history(iter) = cost_trial;
            break;
        end
    else
        lambda = min(lambda * 10, lambda_max);
    end

    if lambda >= lambda_max
        warning('solve_ga_fgo_lm:LambdaOverflow', ...
            'Lambda reached the upper limit. Optimization stopped.');
        break;
    end
end

% Trim cost history
valid_idx = cost_history > 0;
cost_history = cost_history(valid_idx);

%% ------------------------------------------------------------------------
%  Unpack optimized states
%  ------------------------------------------------------------------------
position_ecef = zeros(N, 3);
clock_bias_m = zeros(N, 1);

for k = 1:N
    idx = state_index(k);
    position_ecef(k, :) = x(idx.pos)';
    clock_bias_m(k) = x(idx.cb);
end

%% ------------------------------------------------------------------------
%  Output
%  ------------------------------------------------------------------------
result = struct();
result.position_ecef = position_ecef;
result.clock_bias_m = clock_bias_m;
result.num_epochs = N;
result.num_iterations = numel(cost_history);
result.converged = converged;
result.final_cost = cost_history(end);
result.cost_history = cost_history;
result.state_vector = x;

if verbose
    fprintf('[solve_ga_fgo_lm] Finished. converged = %d, iterations = %d, final cost = %.6e\n', ...
        result.converged, result.num_iterations, result.final_cost);
end

end

%% =========================================================================
%  Local functions
%  =========================================================================

function validate_input_data(data, cfg)

if ~isfield(data, 'initial_pos_ecef') || isempty(data.initial_pos_ecef)
    error('solve_ga_fgo_lm:MissingInitialPosition', ...
        'data.initial_pos_ecef is required.');
end

if size(data.initial_pos_ecef, 2) ~= 3
    error('solve_ga_fgo_lm:InvalidInitialPositionShape', ...
        'data.initial_pos_ecef must be of size [N x 3].');
end

N = size(data.initial_pos_ecef, 1);

if ~isfield(data, 'pr_obs') || isempty(data.pr_obs)
    error('solve_ga_fgo_lm:MissingPseudorangeData', ...
        'data.pr_obs is required.');
end

if ~iscell(data.pr_obs) || numel(data.pr_obs) ~= N
    error('solve_ga_fgo_lm:InvalidPseudorangeCell', ...
        'data.pr_obs must be a cell array with length N.');
end

if isfield(data, 'initial_clk_bias_m') && ~isempty(data.initial_clk_bias_m)
    if numel(data.initial_clk_bias_m) ~= N
        error('solve_ga_fgo_lm:InvalidClockBiasLength', ...
            'data.initial_clk_bias_m must have length N.');
    end
end

if isfield(data, 'tdcp_obs') && ~isempty(data.tdcp_obs)
    if ~iscell(data.tdcp_obs) || numel(data.tdcp_obs) ~= max(N - 1, 0)
        error('solve_ga_fgo_lm:InvalidTDCPCell', ...
            'data.tdcp_obs must be a cell array with length N-1.');
    end
end

window_length = get_cfg_value(cfg, {'fgo', 'window_length'}, N);
if N > window_length
    warning('solve_ga_fgo_lm:WindowLengthMismatch', ...
        ['The current data window contains %d epochs, which is larger than ', ...
         'cfg.fgo.window_length = %d.'], N, window_length);
end

end

function [r, J, w, meta] = build_system(x, x0, data, cfg)

N = size(data.initial_pos_ecef, 1);
state_dim = 4 * N;

r_list = {};
J_list = {};
sigma_list = {};
type_list = {};

%% ------------------------------------------------------------------------
%  Prior factor on the first state
%  ------------------------------------------------------------------------
sigma_pos_prior = get_cfg_value(cfg, {'fgo', 'sigma_pos_prior'}, [10; 10; 10]);
sigma_clk_prior = get_cfg_value(cfg, {'fgo', 'sigma_clk_prior'}, 100);

idx1 = state_index(1);

r_prior_pos = x(idx1.pos) - x0(idx1.pos);
J_prior_pos = sparse(3, state_dim);
J_prior_pos(:, idx1.pos) = speye(3);

r_list{end+1,1} = r_prior_pos;
J_list{end+1,1} = J_prior_pos;
sigma_list{end+1,1} = sigma_pos_prior(:);
type_list{end+1,1} = repmat("prior", 3, 1);

r_prior_clk = x(idx1.cb) - x0(idx1.cb);
J_prior_clk = sparse(1, state_dim);
J_prior_clk(:, idx1.cb) = 1;

r_list{end+1,1} = r_prior_clk;
J_list{end+1,1} = J_prior_clk;
sigma_list{end+1,1} = sigma_clk_prior;
type_list{end+1,1} = "prior";

%% ------------------------------------------------------------------------
%  Pseudorange factors
%  ------------------------------------------------------------------------
num_pr_factors = 0;

default_sigma_pr = get_cfg_value(cfg, {'fgo', 'sigma_pr'}, 5.0);

for k = 1:N
    if isempty(data.pr_obs{k})
        continue;
    end

    obs = data.pr_obs{k};

    if ~isfield(obs, 'sat_pos_ecef') || ~isfield(obs, 'rho_m')
        error('solve_ga_fgo_lm:InvalidPseudorangeObservation', ...
            'Each data.pr_obs{k} must contain sat_pos_ecef and rho_m.');
    end

    sat_pos = obs.sat_pos_ecef;
    rho_m = obs.rho_m(:);

    if size(sat_pos, 1) ~= numel(rho_m) || size(sat_pos, 2) ~= 3
        error('solve_ga_fgo_lm:PseudorangeSizeMismatch', ...
            'sat_pos_ecef must be [M x 3] and rho_m must be [M x 1].');
    end

    if isfield(obs, 'sigma_m') && ~isempty(obs.sigma_m)
        sigma_m = obs.sigma_m(:);
    else
        sigma_m = default_sigma_pr * ones(size(rho_m));
    end

    idx = state_index(k);
    p = x(idx.pos);
    cb = x(idx.cb);

    M = size(sat_pos, 1);
    for j = 1:M
        s = sat_pos(j, :)';
        [res_j, jac_pos_j, jac_cb_j] = pseudorange_factor(p, cb, s, rho_m(j));

        Jj = sparse(1, state_dim);
        Jj(:, idx.pos) = jac_pos_j;
        Jj(:, idx.cb) = jac_cb_j;

        r_list{end+1,1} = res_j;
        J_list{end+1,1} = Jj;
        sigma_list{end+1,1} = sigma_m(j);
        type_list{end+1,1} = "pr";

        num_pr_factors = num_pr_factors + 1;
    end
end

%% ------------------------------------------------------------------------
%  TDCP factors
%  ------------------------------------------------------------------------
num_tdcp_factors = 0;

default_sigma_tdcp = get_cfg_value(cfg, {'fgo', 'sigma_tdcp'}, 0.05);

if isfield(data, 'tdcp_obs') && ~isempty(data.tdcp_obs)
    for k = 1:(N - 1)
        if isempty(data.tdcp_obs{k})
            continue;
        end

        obs = data.tdcp_obs{k};

        required_fields = {'sat_pos_prev_ecef', 'sat_pos_curr_ecef', 'tdcp_m'};
        for i = 1:numel(required_fields)
            if ~isfield(obs, required_fields{i})
                error('solve_ga_fgo_lm:InvalidTDCPObservation', ...
                    'Each data.tdcp_obs{k} must contain %s.', required_fields{i});
            end
        end

        sat_prev = obs.sat_pos_prev_ecef;
        sat_curr = obs.sat_pos_curr_ecef;
        tdcp_m = obs.tdcp_m(:);

        if size(sat_prev,1) ~= numel(tdcp_m) || size(sat_curr,1) ~= numel(tdcp_m) ...
                || size(sat_prev,2) ~= 3 || size(sat_curr,2) ~= 3
            error('solve_ga_fgo_lm:TDCPSizeMismatch', ...
                'TDCP satellite positions must be [L x 3] and tdcp_m must be [L x 1].');
        end

        if isfield(obs, 'sigma_m') && ~isempty(obs.sigma_m)
            sigma_m = obs.sigma_m(:);
        else
            sigma_m = default_sigma_tdcp * ones(size(tdcp_m));
        end

        idx1 = state_index(k);
        idx2 = state_index(k + 1);

        p1 = x(idx1.pos);
        cb1 = x(idx1.cb);
        p2 = x(idx2.pos);
        cb2 = x(idx2.cb);

        L = size(sat_prev, 1);
        for j = 1:L
            s1 = sat_prev(j, :)';
            s2 = sat_curr(j, :)';

            [res_j, jac_p1_j, jac_cb1_j, jac_p2_j, jac_cb2_j] = ...
                tdcp_factor(p1, cb1, p2, cb2, s1, s2, tdcp_m(j));

            Jj = sparse(1, state_dim);
            Jj(:, idx1.pos) = jac_p1_j;
            Jj(:, idx1.cb)  = jac_cb1_j;
            Jj(:, idx2.pos) = jac_p2_j;
            Jj(:, idx2.cb)  = jac_cb2_j;

            r_list{end+1,1} = res_j;
            J_list{end+1,1} = Jj;
            sigma_list{end+1,1} = sigma_m(j);
            type_list{end+1,1} = "tdcp";

            num_tdcp_factors = num_tdcp_factors + 1;
        end
    end
end

%% ------------------------------------------------------------------------
%  Stack system
%  ------------------------------------------------------------------------
r = vertcat(r_list{:});
J = vertcat(J_list{:});
sigma = vertcat(sigma_list{:});
types = vertcat(type_list{:});

w = build_weights(r, sigma, types, cfg);

meta = struct();
meta.num_pr_factors = num_pr_factors;
meta.num_tdcp_factors = num_tdcp_factors;

end

function w = build_weights(r, sigma, types, cfg)

sigma = max(sigma(:), 1e-8);
base_w = 1 ./ (sigma .^ 2);

robust_loss = lower(string(get_cfg_value(cfg, {'fgo', 'robust_loss'}, 'cauchy')));
cauchy_scale = get_cfg_value(cfg, {'fgo', 'cauchy_scale'}, 2.0);

w = base_w;

switch robust_loss
    case "cauchy"
        % Standardized residual
        t = r ./ sigma;
        robust_w = 1 ./ (1 + (t ./ cauchy_scale).^2);

        % Keep prior factors purely quadratic
        is_prior = (types == "prior");
        robust_w(is_prior) = 1.0;

        w = base_w .* robust_w;

    case "none"
        % Do nothing

    otherwise
        warning('solve_ga_fgo_lm:UnknownRobustLoss', ...
            'Unknown robust loss "%s". Quadratic loss is used.', robust_loss);
end

end

function cost = compute_cost(r, w)
cost = sum(w .* (r .^ 2));
end

function [res, jac_pos, jac_cb] = pseudorange_factor(p, cb, sat_pos, rho_meas)

diff_vec = p - sat_pos;
range = norm(diff_vec);

if range < 1e-8
    error('solve_ga_fgo_lm:DegenerateGeometry', ...
        'Receiver position is too close to the satellite position.');
end

pred = range + cb;
res = pred - rho_meas;

jac_pos = (diff_vec / range)';
jac_cb = 1.0;

end

function [res, jac_p1, jac_cb1, jac_p2, jac_cb2] = tdcp_factor( ...
    p1, cb1, p2, cb2, sat_pos_prev, sat_pos_curr, tdcp_meas)

diff1 = p1 - sat_pos_prev;
diff2 = p2 - sat_pos_curr;

range1 = norm(diff1);
range2 = norm(diff2);

if range1 < 1e-8 || range2 < 1e-8
    error('solve_ga_fgo_lm:DegenerateTDCPGeometry', ...
        'Receiver position is too close to the satellite position.');
end

pred = (range2 - range1) + (cb2 - cb1);
res = pred - tdcp_meas;

jac_p1 = -(diff1 / range1)';
jac_cb1 = -1.0;
jac_p2 =  (diff2 / range2)';
jac_cb2 =  1.0;

end

function idx = state_index(k)
base = 4 * (k - 1);
idx.pos = base + (1:3);
idx.cb  = base + 4;
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