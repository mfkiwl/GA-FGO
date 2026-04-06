function cfg = config_default()
%CONFIG_DEFAULT Default configuration for the GA-FGO MATLAB demo.
%
% This configuration is designed for the minimal public release of the
% core MATLAB code corresponding to:
%   1) fisheye-image-based LOS/NLOS classification,
%   2) geometry-adaptive NLOS handling,
%   3) sliding-window factor graph optimization with pseudorange and TDCP.
%
% Notes:
% - The current public version is intended for code organization and demo.
% - Full street-level fisheye data are not included in the repository.
% - Please modify the paths below according to your local environment.

%% ------------------------------------------------------------------------
%  Basic repository paths
%  ------------------------------------------------------------------------
cfg = struct();

cfg.repo.name = 'ga-fgo-matlab';
cfg.repo.version = 'v0.1-minimal';

% config_default.m is inside: <repo>/config/
cfg.paths.repo_root = fileparts(fileparts(mfilename('fullpath')));
cfg.paths.config_dir = fullfile(cfg.paths.repo_root, 'config');
cfg.paths.sample_data_dir = fullfile(cfg.paths.repo_root, 'sample_data');
cfg.paths.preprocess_dir = fullfile(cfg.paths.repo_root, 'preprocess');
cfg.paths.projection_dir = fullfile(cfg.paths.repo_root, 'projection');
cfg.paths.handling_dir = fullfile(cfg.paths.repo_root, 'handling');
cfg.paths.fgo_dir = fullfile(cfg.paths.repo_root, 'fgo');

%% ------------------------------------------------------------------------
%  External dependencies
%  ------------------------------------------------------------------------


cfg.deps = struct();

cfg.deps.use_gtsam = true;
cfg.deps.gtsam_toolbox_path = '/usr/local/ga-fgo/gtsam_toolbox/';     
cfg.deps.use_gtsam_gnss = true;
cfg.deps.gtsam_gnss_path = '/usr/local/ga-fgo/gnss_gnss/';        % fill if needed
cfg.deps.use_matrtklib = false;
cfg.deps.matrtklib_path = '/usr/local/ga-fgo/matrtklib/';         % optional
cfg.deps.auto_add_paths = true;

if cfg.deps.auto_add_paths
    local_dirs = { ...
        cfg.paths.preprocess_dir, ...
        cfg.paths.projection_dir, ...
        cfg.paths.handling_dir, ...
        cfg.paths.fgo_dir ...
        };
    for i = 1:numel(local_dirs)
        if exist(local_dirs{i}, 'dir')
            addpath(local_dirs{i});
        end
    end

    if ~isempty(cfg.deps.gtsam_toolbox_path) && exist(cfg.deps.gtsam_toolbox_path, 'dir')
        addpath(cfg.deps.gtsam_toolbox_path);
    end

    if ~isempty(cfg.deps.gtsam_gnss_path) && exist(cfg.deps.gtsam_gnss_path, 'dir')
        addpath(cfg.deps.gtsam_gnss_path);
    end

    if ~isempty(cfg.deps.matrtklib_path) && exist(cfg.deps.matrtklib_path, 'dir')
        addpath(cfg.deps.matrtklib_path);
    end
end

%% ------------------------------------------------------------------------
%  Demo data settings
%  ------------------------------------------------------------------------
cfg.data = struct();

cfg.data.use_sample_data = true;
cfg.data.sequence_name = 'demo_sequence';
cfg.data.start_epoch = 1;
cfg.data.end_epoch = 100;
cfg.data.sampling_interval = 1.0; % second

% Example file paths in the public repository
cfg.data.fisheye_image = fullfile(cfg.paths.sample_data_dir, ...
    'fisheye_preview', 'fisheye_001.jpg');
cfg.data.camera_param_file = fullfile(cfg.paths.sample_data_dir, ...
    'camera_params', 'camera_params_e4p.mat');

% Optional demo observation excerpts
cfg.data.pseudorange_file = fullfile(cfg.paths.sample_data_dir, ...
    'gnss_excerpt', 'pseudorange_excerpt.csv');
cfg.data.carrier_phase_file = fullfile(cfg.paths.sample_data_dir, ...
    'gnss_excerpt', 'carrier_phase_excerpt.csv');
cfg.data.sat_info_file = fullfile(cfg.paths.sample_data_dir, ...
    'gnss_excerpt', 'sat_info_excerpt.csv');
cfg.data.heading_file = fullfile(cfg.paths.sample_data_dir, ...
    'imu_excerpt', 'heading_excerpt.csv');

%% ------------------------------------------------------------------------
%  Fisheye camera model
%  ------------------------------------------------------------------------
% Paper model:
%   theta_img = theta_sat_az + heading
%   gamma = pi/2 - elevation
%   r = 2*f*tan(gamma/2)
cfg.camera = struct();

cfg.camera.model = 'stereographic_fisheye';
cfg.camera.image_width = 1920;
cfg.camera.image_height = 1080;

% Principal point (editable)
cfg.camera.cx = cfg.camera.image_width  / 2;
cfg.camera.cy = cfg.camera.image_height / 2;

% Effective focal length in pixels (editable demo value)
cfg.camera.f = 520.0;

% Heading convention
cfg.camera.heading_unit = 'deg';
cfg.camera.azimuth_unit = 'deg';
cfg.camera.elevation_unit = 'deg';

% Whether to load calibrated camera parameters from .mat
cfg.camera.use_calibrated_params = true;

%% ------------------------------------------------------------------------
%  Otsu-based Sky-mask segmentation
%  ------------------------------------------------------------------------
cfg.seg = struct();

cfg.seg.gray_method = 'rgb2gray';
cfg.seg.threshold_method = 'otsu';
cfg.seg.apply_median_filter = true;
cfg.seg.median_filter_size = [5 5];

% Binary mask convention
cfg.seg.sky_value = 1;
cfg.seg.non_sky_value = 0;

%% ------------------------------------------------------------------------
%  LOS/NLOS classification
%  ------------------------------------------------------------------------
cfg.classifier = struct();

cfg.classifier.out_of_image_as_nlos = true;
cfg.classifier.round_pixel_method = 'nearest';
cfg.classifier.enable_visual_check = true;

%% ------------------------------------------------------------------------
%  Geometry-adaptive NLOS handling
%  ------------------------------------------------------------------------
cfg.handling = struct();

% Thresholds from the manuscript
cfg.handling.T1_num_los = 8;
cfg.handling.T2_gdop = 4.0;

% Mode names
cfg.handling.mode_remove_nlos = 'remove_nlos';
cfg.handling.mode_downweight_nlos = 'downweight_nlos';

% Weight model parameters from the manuscript
cfg.handling.weight_model = 'paper_eq5';
cfg.handling.s0 = 10.0;   % dB
cfg.handling.sa = 31.0;   % dB
cfg.handling.s1 = 47.0;   % dB
cfg.handling.k_los = 1.0;
cfg.handling.k_nlos = 1.6;

% Safety bounds
cfg.handling.min_weight = 1e-3;
cfg.handling.max_weight = 1.0;

%% ------------------------------------------------------------------------
%  GNSS observation and correction models
%  ------------------------------------------------------------------------
cfg.gnss = struct();

cfg.gnss.systems = {'GPS', 'BDS', 'GAL'};
cfg.gnss.use_pseudorange = true;
cfg.gnss.use_tdcp = true;

cfg.gnss.use_klobuchar = true;
cfg.gnss.use_saastamoinen = true;
cfg.gnss.use_niell_mapping = true;

% Simple demo masks / checks
cfg.gnss.min_elevation_deg = 10.0;
cfg.gnss.min_snr_dbhz = 20.0;

%% ------------------------------------------------------------------------
%  TDCP settings
%  ------------------------------------------------------------------------
cfg.tdcp = struct();

cfg.tdcp.enabled = true;
cfg.tdcp.require_consecutive_epochs = true;
cfg.tdcp.check_cycle_slip = true;
cfg.tdcp.max_tdcp_abs_m = 100.0;      % loose demo bound
cfg.tdcp.default_sigma_m = 0.05;      % editable demo value

%% ------------------------------------------------------------------------
%  Factor graph optimization settings
%  ------------------------------------------------------------------------
cfg.fgo = struct();

cfg.fgo.window_length = 15;
cfg.fgo.state_definition = '[position_ecef(3), clock_bias(1)]';
cfg.fgo.use_fixed_lag = true;
cfg.fgo.use_schur_marginalization = true;

% Optimizer
cfg.fgo.optimizer = 'LM';
cfg.fgo.max_iterations = 100;
cfg.fgo.relative_error_tol = 1e-6;
cfg.fgo.absolute_error_tol = 1e-6;

% Robust loss
cfg.fgo.robust_loss = 'cauchy';
cfg.fgo.cauchy_scale = 2.0;

% Initial standard deviations (editable demo values)
cfg.fgo.sigma_pos_prior = [10; 10; 10];  % meter
cfg.fgo.sigma_clk_prior = 100;           % meter-equivalent
cfg.fgo.sigma_pr = 5.0;                  % meter
cfg.fgo.sigma_tdcp = cfg.tdcp.default_sigma_m;

%% ------------------------------------------------------------------------
%  Output / debug settings
%  ------------------------------------------------------------------------
cfg.output = struct();

cfg.output.verbose = true;
cfg.output.show_figures = true;
cfg.output.save_results = false;
cfg.output.results_dir = fullfile(cfg.paths.repo_root, 'results');

if cfg.output.save_results && ~exist(cfg.output.results_dir, 'dir')
    mkdir(cfg.output.results_dir);
end

%% ------------------------------------------------------------------------
%  Sanity checks
%  ------------------------------------------------------------------------
if cfg.data.end_epoch < cfg.data.start_epoch
    error('config_default:InvalidEpochRange', ...
        'end_epoch must be >= start_epoch.');
end

if cfg.fgo.window_length < 2
    error('config_default:InvalidWindowLength', ...
        'window_length must be at least 2.');
end

if cfg.handling.T1_num_los < 4
    warning('config_default:SmallT1', ...
        'T1_num_los is very small. Please check if this is intentional.');
end

if cfg.output.verbose
    fprintf('[config_default] Configuration loaded successfully.\n');
    fprintf('  Repo root     : %s\n', cfg.paths.repo_root);
    fprintf('  Window length : %d\n', cfg.fgo.window_length);
    fprintf('  T1 / T2       : %d / %.2f\n', ...
        cfg.handling.T1_num_los, cfg.handling.T2_gdop);
    fprintf('  Use TDCP      : %d\n', cfg.tdcp.enabled);
    fprintf('  Optimizer     : %s\n', cfg.fgo.optimizer);
end

end