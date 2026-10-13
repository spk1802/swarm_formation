function out = analyze_formation_csv(csvFile, varargin)
% ANALYZE_FORMATION_CSV  Convergence dashboards for a formation_logger.py CSV
% (Basak & Ghosh, ICC 2025): leader convergence onto the path, follower
% formation convergence, follower motion, an overview, and the convergence
% type (asymptotic leader vs finite-time followers).
%
% Usage (MATLAB Online):
%   1. Upload the CSV AND the run's run_config.txt to the same folder.
%   2. analyze_formation_csv('formation_20261005_184802_v2_full_n10e10.csv')
%      analyze_formation_csv                                  % file picker
%      out = analyze_formation_csv(file, 'Path', 'sinusoid', 'Offset', [10 10])
%   3. Download <csvname>_analysis.png / .pdf from the CSV's folder
%      (Current Folder panel -> right-click -> Download).
%
% The CSV has no reference path, so the leader's cross-track error d is
% rebuilt here: path type, offset and sinusoid shape come from run_config.txt
% next to the CSV (leader=..., path_offset=..., path_offset_north=...,
% amplitude=..., wavelength=...) unless given as arguments. Shape parameters
% the run left at "<node default>" use the path_leader node's defaults. The
% leader always starts at the origin; the offset shifts the PATH (same as
% start_swarm.sh --path-offset E or N,E).
%
% Options (name-value):
%   'Path'           'sinusoid' | 'figure8' | 'circle' | 'straight_line'
%   'Offset'         East shift [m], or [North East] like --path-offset N,E
%   'OffsetNorth'    North shift [m] (alternative to the 2-element 'Offset')
%                    (both 0 = exact mode)
%   'Amplitude',10   'Wavelength',130    sinusoid y = A (cos(2 pi x/lambda) - 1)
%                                        (run_config.txt values win over these
%                                        defaults; an explicit argument wins over both)
%   'F8Length',60    'F8Width',20        figure8 shape
%   'Radius',20      'Direction',1       circle shape
%   'K',0.3          'ChiInf',pi/2       leader vector field (for chi_d)
%   'DTol',0.10      follower D_i band [m]
%   'ChiTol',0.15    follower heading-residual band [rad]
%   'LeaderDTol',0.10     leader |d| band [m]
%   'LeaderChiTol',0.05   leader heading band [rad]
%   'LeaderDLog',false    true = leader |d| on a log axis instead of linear d
%   'Export',true         save all figures: <csvname>_analysis.png (one
%                         image, 2-column grid) and <csvname>_analysis.pdf (one
%                         figure per page, vector graphics)
%   'ExportDir',''        output folder ('' = the CSV's folder)
%   'Report',true         true = four clean result figures (trajectories, leader,
%                         followers, finite-time + t_s table); false = the five
%                         detailed diagnostic figures
%   Figure 5 (follower heading law; not logged, defaults = follower_controller.py):
%   'Eta',pi/4            reaching gain eta
%   'P',3/5               reaching exponent p = n/m (0 < p < 1)
%   'Ky',1.2              vector-field gain k_y
%   'FollowerChiInf',pi/2 follower chi_inf
%   'EnableDelta',[]      true/false: was Delta in the controller's chi_d?
%                         ([] = enable_delta= from run_config.txt, else true)
%   'ZoomT',30            Figure 5 follower window [s after leader start]
%   'TightK',3            tight band = TightK * steady-state RMS of chi_tilde
%
% Convergence times are measured from leader motion onset (t = 0):
%   reach   first time |error| <= tol
%   settle  first time after which |error| stays <= tol to the end of the log
%           (0 = never left the band, "not conv." = still outside at the end)
% Follower bands must sit above the SITL noise floor (99th percentile ~0.08 m
% for D_i, ~0.07 rad heading residual) or "settle" lands on the last noise
% spike. The leader is software (noise-free), so its bands can be tight.
%
% Leader errors
%   d                       signed cross-track distance (left of path = +)
%   chi_l - chi_p           heading relative to the path tangent
%   chi_l - chi_d           vector-field heading error (Lemma 1: reaches 0 in
%                           finite time, after which d decays exponentially)
% Exponential fit: once chi_l - chi_d has settled, d ~ d0*exp(-t/tau) is
% fitted where |d| > 1 mm. Linearised vector field: d_dot = -v*K*(2*chi_inf/pi)*d,
% so the theoretical tau = pi / (2*chi_inf*v*K) (= 1/(v*K) for chi_inf = pi/2).
% Exact-mode note: logs from the old sinusoid_leader node (before path_leader)
% show |d| up to ~1e-3 m -- that node integrated position with forward Euler
% and drifts ~1 mm off the curve. path_leader places the leader on r(u).
% Follower errors
%   D_i                     formation error
%   chi_i - chi_d           heading error of the reaching law (Figures 3 and 5),
%                           chi_d = chi_l + Delta_i - chi_inf (2/pi) atan(k_y e_y)
%                           rebuilt from <f>_chi_err_res and <f>_eyi (chi_d is not logged)
%                           Finite time: t_s = |chi_tilde(0)|^(1-p) / (eta (1-p))
%   chi_i - chi_l - Delta_i old residual (chi_err_res; falls back to chi_err), drawn
%                           thin dashed in Figure 3 for comparison

    if nargin < 1 || isempty(csvFile)
        [f, p] = uigetfile('*.csv', 'Select formation CSV');
        if isequal(f, 0); out = []; return; end
        csvFile = fullfile(p, f);
    end

    pos  = @(x) isnumeric(x) && isscalar(x) && x > 0;
    num  = @(x) isnumeric(x) && isscalar(x);
    num2 = @(x) isnumeric(x) && (numel(x) == 1 || numel(x) == 2);
    tf   = @(x) (islogical(x) || isnumeric(x)) && isscalar(x);
    tf0  = @(x) isempty(x) || tf(x);
    frac = @(x) isnumeric(x) && isscalar(x) && x > 0 && x < 1;
    str  = @(s) ischar(s) || isstring(s);
    ip = inputParser;
    addParameter(ip, 'Path', '', str);
    addParameter(ip, 'Offset', NaN, num2);
    addParameter(ip, 'OffsetNorth', NaN, num);
    addParameter(ip, 'Amplitude', 10, pos);
    addParameter(ip, 'Wavelength', 130, pos);
    addParameter(ip, 'F8Length', 60, pos);
    addParameter(ip, 'F8Width', 20, pos);
    addParameter(ip, 'Radius', 20, pos);
    addParameter(ip, 'Direction', 1, num);
    addParameter(ip, 'K', 0.3, pos);
    addParameter(ip, 'ChiInf', pi/2, pos);
    addParameter(ip, 'DTol', 0.10, pos);
    addParameter(ip, 'ChiTol', 0.15, pos);
    addParameter(ip, 'LeaderDTol', 0.10, pos);
    addParameter(ip, 'LeaderChiTol', 0.05, pos);
    addParameter(ip, 'LeaderDLog', false, tf);
    addParameter(ip, 'Export', true, tf);
    addParameter(ip, 'ExportDir', '', str);
    addParameter(ip, 'Eta', pi/4, pos);
    addParameter(ip, 'P', 3/5, frac);
    addParameter(ip, 'Ky', 1.2, pos);
    addParameter(ip, 'FollowerChiInf', pi/2, pos);
    addParameter(ip, 'EnableDelta', [], tf0);
    addParameter(ip, 'ZoomT', 30, pos);
    addParameter(ip, 'TightK', 3, pos);
    addParameter(ip, 'Report', true, tf);
    parse(ip, varargin{:});
    o = ip.Results;

    T = readtable(csvFile, 'VariableNamingRule', 'preserve');
    if height(T) == 0
        error('CSV "%s" has column headers but no data rows.', csvFile);
    end
    vars = T.Properties.VariableNames;

    % ---- time axis: t = 0 at leader motion onset ---------------------------
    iOn = find(T.leader_speed > 1e-3, 1);
    if isempty(iOn)
        error('The leader never moved in %s; nothing to analyse.', csvFile);
    end
    t = T.t - T.t(iOn);
    nSS = max(1, round(0.2 * numel(t)));        % last 20% = steady state

    % ---- followers ---------------------------------------------------------
    foll = erase(vars(endsWith(vars, '_Di')), '_Di');
    if isempty(foll); error('No follower columns (*_Di) in %s', csvFile); end
    nF = numel(foll);
    cmap = lines(nF);
    col = @(name, suffix) T.(sprintf('%s_%s', name, suffix));
    if any(strcmp(vars, [foll{1} '_chi_err_res']))
        chiSuf = 'chi_err_res'; chiLbl = '\chi_i - \chi_l - \Delta_i (rad)';
    else
        chiSuf = 'chi_err';     chiLbl = '\chi_i - \chi_l (rad)';
        warning('No *_chi_err_res column: using chi_err (settles at Delta on curves, not 0).');
    end

    % ---- reference path and leader errors ----------------------------------
    [pathName, offN, offE, src, cfg] = resolve_path(csvFile, o);
    % sinusoid shape: run_config.txt overrides the defaults, an argument overrides both
    if isfield(cfg, 'amplitude') && ismember('Amplitude', ip.UsingDefaults)
        o.Amplitude = cfg.amplitude;
    end
    if isfield(cfg, 'wavelength') && ismember('Wavelength', ip.UsingDefaults)
        o.Wavelength = cfg.wavelength;
    end
    exactMode = (offN == 0 && offE == 0);
    offTxt = sprintf('offset %+.1f m N, %+.1f m E', offN, offE);
    P = make_path(pathName, offN, offE, o);
    n = height(T);
    dL = nan(n, 1); chiP = nan(n, 1); u = 0;
    for k = 1:n
        u = closest_point(P, T.leader_x(k), T.leader_y(k), u);
        [r, r1] = path_eval(P, u);
        chiP(k) = atan2(r1(2), r1(1));
        dL(k) = -(T.leader_x(k) - r(1)) * sin(chiP(k)) ...
                + (T.leader_y(k) - r(2)) * cos(chiP(k));
    end
    ePath  = wrapPi(T.leader_chi - chiP);
    chiD   = chiP - o.ChiInf * (2/pi) * atan(o.K * dL);
    eTilde = wrapPi(T.leader_chi - chiD);
    shapeTxt = '';
    if strcmp(pathName, 'sinusoid')
        shapeTxt = sprintf(', A = %g m, lambda = %g m', o.Amplitude, o.Wavelength);
    end
    fprintf('\nReference path: %s%s, %s (%s)\n', pathName, shapeTxt, offTxt, src);
    late = abs(dL(t > 60));
    pathMismatch = ~isempty(late) && max(late) > 0.5;
    if pathMismatch
        warning(['Leader is up to %.1f m from the assumed path after t = 60 s: ', ...
                 'wrong ''Path'' / ''Offset'' / ''OffsetNorth'' / shape parameters?'], max(late));
    end

    % ---- convergence times -------------------------------------------------
    L = struct();
    [L.reach_d,  L.settle_d]  = conv_time(t, dL,     o.LeaderDTol);
    [L.reach_hp, L.settle_hp] = conv_time(t, ePath,  o.LeaderChiTol);
    [L.reach_ht, L.settle_ht] = conv_time(t, eTilde, o.LeaderChiTol);

    % exponential decay of d after the vector-field heading has settled
    [L.tau, fitT, fitD, L.tau_window] = exp_fit(t, dL, L.settle_ht);
    vNom = median(T.leader_speed(t > 10), 'omitnan');
    L.tau_theory = pi / (2 * o.ChiInf * vNom * o.K);

    % ---- follower heading error of the reaching law ------------------------
    % chi_d is not logged: rebuilt from logged columns and the controller gains
    %   chi_d     = chi_l + Delta_i - chi_inf (2/pi) atan(k_y e_y)
    %   chi_tilde = wrap( (chi_i - chi_l - Delta_i) + chi_inf (2/pi) atan(k_y e_y) )
    %     chi_i - chi_l - Delta_i  <- <f>_chi_err_res  (logger: wrap(chi_err - delta))
    %     chi_i - chi_l            <- <f>_chi_err      (run with enable_delta=false:
    %                                                   the controller's chi_d has no Delta)
    %     e_y                      <- <f>_eyi          (= y_L - g_y, the controller's e_y)
    % k_y, chi_inf (and eta, p for Figure 5) are not logged: defaults are the
    % follower_controller.py constants (self.k_y, self.chi_inf, self.eta, self.n/self.m).
    useDelta = o.EnableDelta;
    srcDelta = 'argument';
    if isempty(useDelta)
        if isfield(cfg, 'enable_delta'); useDelta = cfg.enable_delta; srcDelta = 'run_config.txt';
        else; useDelta = true; srcDelta = 'default'; end
    end
    useDelta = logical(useDelta);
    chiT = nan(n, nF);
    for i = 1:nF
        if ~useDelta
            base = col(foll{i}, 'chi_err');                          % <f>_chi_err
        elseif any(strcmp(vars, [foll{i} '_chi_err_res']))
            base = col(foll{i}, 'chi_err_res');                      % <f>_chi_err_res
        elseif any(strcmp(vars, [foll{i} '_delta']))
            base = wrapPi(col(foll{i}, 'chi_err') - col(foll{i}, 'delta'));  % <f>_chi_err - <f>_delta
        else
            error('enable_delta=true but no %s_chi_err_res or %s_delta column.', foll{i}, foll{i});
        end
        chiT(:, i) = wrapPi(base + o.FollowerChiInf * (2/pi) ...
                            * atan(o.Ky * col(foll{i}, 'eyi')));      % <f>_eyi
    end
    fprintf(['Follower heading error chi_i - chi_d rebuilt (chi_d not logged): ', ...
             'k_y = %.2f, chi_inf = %.4f, Delta %s (%s)\n'], o.Ky, o.FollowerChiInf, ...
            string(useDelta), srcDelta);

    rD = nan(nF, 1); sD = rD; rC = rD; sC = rD; sCold = rD;
    for i = 1:nF
        [rD(i), sD(i)] = conv_time(t, col(foll{i}, 'Di'),   o.DTol);
        [rC(i), sC(i)] = conv_time(t, chiT(:, i),           o.ChiTol);
        [~, sCold(i)]  = conv_time(t, col(foll{i}, chiSuf), o.ChiTol);
    end
    formD = slowest(sD); formC = slowest(sC);
    blue = [0 0.45 0.74];
    m = t >= 0;                                     % after leader start (console stats)
    % ---- finite-time table (Figure 5 / report figure 4 and the console) ----
    bandW = o.ChiTol;                               % the Figure 3 heading band
    ss = chiT(end-nSS+1:end, :);                    % steady state = last 20%
    bandT = o.TightK * sqrt(mean(ss(:).^2, 'omitnan'));
    pE = 1 - o.P;
    c0 = chiT(iOn, :).';                            % chi_tilde at leader start (t = 0)
    tS = abs(c0).^pE / (o.Eta * pE);                % ideal time to chi_tilde = 0
    tSW = max(0, abs(c0).^pE - bandW^pE) / (o.Eta * pE);   % ideal time into the band
    tEW = sC;                                       % measured: enters the band and stays
                                                    % to the end of the log (= Figure 3)
    res = csv_resolution(csvFile);                  % floor for log10(0)

    % table rows, shared by the figure and the console
    tblHdr = sprintf('%-8s %13s %11s %13s %13s', 'follower', 'chi_tilde(0)', ...
                     't_s pred', sprintf('t_band(%.2f)', bandW), 't_enter meas');
    tblRows = cell(nF, 1);
    for i = 1:nF
        tblRows{i} = sprintf('%-8s %+13.4f %9.2f s %11.2f s %13s', foll{i}, c0(i), ...
                             tS(i), tSW(i), fmt_t2(tEW(i)));
    end

    if o.Report
        % =======================================================================
        % REPORT -- four clean figures with the results only ('Report', false
        % gives the five detailed diagnostic figures below instead)
        % =======================================================================
        figs = gobjects(4, 1);
        tStart = -5;                                    % show a little of the hold
        if exactMode || isnan(L.settle_d) || L.settle_d <= 0
            tLead = t(end);
    else
        tLead = min(t(end), max(60, 2.5 * L.settle_d));
    end

    % ---- R1: trajectories (top-down, North to the right) ------------------
    if any(strcmp(pathName, {'sinusoid', 'straight_line'})); sz = [1400 560];
    else; sz = [1100 800]; end
    fig = report_fig('1 Trajectories', sz);  figs(1) = fig;
    ax = axes(fig); hold(ax, 'on');
    ref = path_samples(P, T.leader_x);
    plot(ax, ref(:, 1), ref(:, 2), ':', 'Color', [.5 .5 .5], 'LineWidth', 1.5, ...
         'DisplayName', 'reference path');
    plot(ax, T.leader_x, T.leader_y, 'k-', 'LineWidth', 2.4, 'DisplayName', 'leader');
    for i = 1:nF
        plot(ax, col(foll{i}, 'x'), col(foll{i}, 'y'), 'LineWidth', 1.3, ...
             'Color', cmap(i, :), 'DisplayName', foll{i});
    end
    plot(ax, T.leader_x(iOn), T.leader_y(iOn), 'ko', 'MarkerFaceColor', 'k', ...
         'MarkerSize', 7, 'DisplayName', 'leader start');
    xlabel(ax, 'North x (m)'); ylabel(ax, 'East y (m)');
    title(ax, sprintf('Trajectories: %s path%s, %s', pathName, shapeTxt, offTxt), ...
          'Interpreter', 'none');
    axis(ax, 'equal'); axis(ax, 'tight'); grid(ax, 'on'); box(ax, 'on');
    set(ax, 'YDir', 'reverse');                     % East down = viewed from above
    legend(ax, 'Location', 'eastoutside', 'Box', 'off');

    % ---- R2: leader convergence -------------------------------------------
    fig = report_fig('2 Leader convergence', [1400 850]);  figs(2) = fig;
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    axs = gobjects(2, 1);

    axs(1) = nexttile(tl); hold(axs(1), 'on');
    plot(axs(1), t, dL, 'k-', 'LineWidth', 2, 'DisplayName', 'd');
    if ~isnan(L.tau)
        plot(axs(1), fitT, fitD, 'r--', 'LineWidth', 2, 'DisplayName', ...
             sprintf('fit d_0 e^{-t/\\tau},  \\tau = %.2f s  (theory %.2f s)', L.tau, L.tau_theory));
    end
    yline(axs(1), [-1 1] * o.LeaderDTol, ':', 'Color', [.5 .5 .5], 'HandleVisibility', 'off');
    ylabel(axs(1), 'cross-track d (m)');
    if exactMode
        title(axs(1), sprintf('Leader cross-track error (exact mode): max |d| = %.1e m', ...
              max(abs(dL(t >= 0)))));
    else
        title(axs(1), sprintf('Leader cross-track error: |d| \\leq %.2f m at %s', ...
              o.LeaderDTol, fmt_t(L.settle_d)));
    end
    legend(axs(1), 'Location', 'southeast', 'Box', 'off');

    axs(2) = nexttile(tl); hold(axs(2), 'on');
    plot(axs(2), t, eTilde, '-', 'Color', blue, 'LineWidth', 2, ...
         'DisplayName', sprintf('\\chi_l - \\chi_d  (vector field), settles %s', fmt_t(L.settle_ht)));
    plot(axs(2), t, ePath, 'k-', 'LineWidth', 1.4, ...
         'DisplayName', sprintf('\\chi_l - \\chi_p  (path tangent), settles %s', fmt_t(L.settle_hp)));
    yline(axs(2), [-1 1] * o.LeaderChiTol, ':', 'Color', [.5 .5 .5], 'HandleVisibility', 'off');
    ylabel(axs(2), 'heading error (rad)');
    title(axs(2), sprintf('Leader heading error (band \\pm%.2f rad)', o.LeaderChiTol));
    xlabel(axs(2), 'time since leader start (s)');
    legend(axs(2), 'Location', 'southeast', 'Box', 'off');
    report_axes(axs, [tStart tLead]);

    % ---- R3: follower convergence -----------------------------------------
    fig = report_fig('3 Follower convergence', [1400 850]);  figs(3) = fig;
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    axs = gobjects(2, 1);

    axs(1) = nexttile(tl); hold(axs(1), 'on');
    for i = 1:nF
        plot(axs(1), t, col(foll{i}, 'Di'), 'LineWidth', 1.1, 'Color', cmap(i, :), ...
             'DisplayName', sprintf('%s  (%s)', foll{i}, fmt_t(sD(i))));       % <f>_Di
    end
    set(axs(1), 'YScale', 'log');
    yline(axs(1), o.DTol, 'k--', 'HandleVisibility', 'off');
    ylabel(axs(1), 'D_i (m)');
    title(axs(1), sprintf('Formation error D_i: all within %.2f m at %s', o.DTol, fmt_t(formD)));
    legend(axs(1), 'Location', 'northeast', 'Box', 'off', 'NumColumns', 2);

    axs(2) = nexttile(tl); hold(axs(2), 'on');
    for i = 1:nF
        plot(axs(2), t, chiT(:, i), 'LineWidth', 1.1, 'Color', cmap(i, :), ...
             'DisplayName', sprintf('%s  (%s)', foll{i}, fmt_t(sC(i))));
    end
    yline(axs(2), [-1 1] * o.ChiTol, 'k--', 'HandleVisibility', 'off');
    ylabel(axs(2), '\chi_i - \chi_d (rad)');
    title(axs(2), sprintf('Heading error \\chi_i - \\chi_d: all within \\pm%.2f rad at %s', ...
          o.ChiTol, fmt_t(formC)));
    xlabel(axs(2), 'time since leader start (s)');
    legend(axs(2), 'Location', 'northeast', 'Box', 'off', 'NumColumns', 2);
    report_axes(axs, [tStart t(end)]);
    yl = max(abs(chiT(t >= 0, :)), [], 'all');
    ylim(axs(2), [-1 1] * max(2 * o.ChiTol, 1.1 * yl));

    % ---- R4: convergence type + t_s table ---------------------------------
    fig = report_fig('4 Finite-time convergence', [1400 850]);  figs(4) = fig;
    tl = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    lg = @(x) log10(max(abs(x), res));
    axs = gobjects(2, 1);

    axs(1) = nexttile(tl, 1, [1 2]); hold(axs(1), 'on');
    plot(axs(1), t, lg(dL), 'k-', 'LineWidth', 1.8, 'DisplayName', '|d|  (m)');
    if ~isnan(L.tau)
        plot(axs(1), fitT, lg(fitD), 'r--', 'LineWidth', 2, 'DisplayName', ...
             sprintf('exponential fit, \\tau = %.2f s', L.tau));
    end
    plot(axs(1), t, lg(eTilde), '-', 'Color', blue, 'LineWidth', 1.4, ...
         'DisplayName', '|\chi_l - \chi_d|  (rad)');
    ylabel(axs(1), 'log_{10}');
    if exactMode
        title(axs(1), 'Leader (exact mode: placed on the path, not steered)');
    else
        title(axs(1), 'Leader: asymptotic (exponential = straight line)');
    end
    xlim(axs(1), [tStart tLead]);
    ylim(axs(1), [log10(res) - 0.3, max(1, log10(max(abs(dL))) + 0.5)]);
    legend(axs(1), 'Location', 'northeast', 'Box', 'off');

    axs(2) = nexttile(tl, 4, [1 2]); hold(axs(2), 'on');
    for i = 1:nF
        plot(axs(2), t, lg(chiT(:, i)), 'LineWidth', 1.2, 'Color', cmap(i, :), ...
             'DisplayName', foll{i});
        ti = linspace(0, tS(i), 200).';
        plot(axs(2), ti, lg(max(0, abs(c0(i))^pE - o.Eta * pE * ti).^(1/pE)), '--', ...
             'Color', cmap(i, :), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
    plot(axs(2), nan, nan, 'k--', 'DisplayName', 'ideal law');
    yline(axs(2), log10(bandW), 'k-', 'LineWidth', 1.2, ...
          'DisplayName', sprintf('band %.2f rad', bandW));
    ylabel(axs(2), 'log_{10}|\chi_i - \chi_d|');
    title(axs(2), sprintf('Followers: finite time, first %g s', o.ZoomT));
    xlabel(axs(2), 'time since leader start (s)');
    xlim(axs(2), [-1 o.ZoomT]);
    ylim(axs(2), [max(log10(res), floor(log10(bandT)) - 1), 0.5]);
    legend(axs(2), 'Location', 'northeast', 'Box', 'off', 'NumColumns', 2);
    for k = 1:2
        grid(axs(k), 'on'); box(axs(k), 'on');
        xline(axs(k), 0, 'k:', 'HandleVisibility', 'off');
    end

    axT = nexttile(tl, 3, [2 1]); axis(axT, 'off');
    rows = cell(nF, 1);
    for i = 1:nF
        rows{i} = sprintf('%-3s %+7.3f %6.2f %6.2f %7s', foll{i}, c0(i), tS(i), tSW(i), ...
                          erase(fmt_t2(tEW(i)), ' s'));
    end
    txt = [{'Finite-time check'; ''; ...
            sprintf('%-3s %7s %6s %6s %7s', '', 'chi(0)', 't_s', 't_band', 't_enter'); ...
            sprintf('%-3s %7s %6s %6s %7s', '', '(rad)', '(s)', '(s)', '(s)')}; rows; ...
           {''; sprintf('eta = %.3f, p = %.2f', o.Eta, o.P); ''; ...
            't_s    = |chi(0)|^(1-p) / (eta(1-p))'; ...
            '         predicted time to 0'; ...
            sprintf('t_band = predicted time to %.2f rad', bandW); ...
            't_enter = measured: enters the band'; ...
            '          and stays to end of log'}];
    text(axT, 0, 1, txt, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'FontName', 'FixedWidth', 'FontSize', 12, 'Interpreter', 'none');

    else
        figs = gobjects(5, 1);

        % =======================================================================
        % FIGURE 1 -- Overview
        % =======================================================================
        fig = figure('Name', '1 Overview', 'Color', 'w');  figs(1) = fig;
        tl = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
        title(tl, sprintf('Overview -- %s path%s, %s', pathName, shapeTxt, offTxt), ...
              'Interpreter', 'none');

        % trajectories: North to the right, East down (top-down view), 2/3 width
        ax = nexttile(tl, [1 2]); hold(ax, 'on');
        ref = path_samples(P, T.leader_x);
        plot(ax, ref(:, 1), ref(:, 2), ':', 'Color', [.5 .5 .5], 'LineWidth', 1.4, ...
             'DisplayName', 'reference path');
        plot(ax, T.leader_x, T.leader_y, 'k-', 'LineWidth', 2, 'DisplayName', 'leader');
        for i = 1:nF
            plot(ax, col(foll{i}, 'x'), col(foll{i}, 'y'), 'LineWidth', 1.2, ...
                 'Color', cmap(i, :), 'DisplayName', foll{i});
        end
        plot(ax, T.leader_x(iOn), T.leader_y(iOn), 'ko', 'MarkerFaceColor', 'k', ...
             'DisplayName', 'leader start');
        xlabel(ax, 'North x (m)'); ylabel(ax, 'East y (m)');
        title(ax, 'Trajectories (paper Fig 4a), North to the right');
        axis(ax, 'equal'); axis(ax, 'tight'); grid(ax, 'on'); box(ax, 'on');
        set(ax, 'YDir', 'reverse');                        % East down = viewed from above
        legend(ax, 'Location', 'bestoutside');

        ax = nexttile(tl); hold(ax, 'on');
        m = t >= 0;
        for i = 1:nF
            xL = col(foll{i}, 'xL'); yL = col(foll{i}, 'yL');
            gx = median(xL - col(foll{i}, 'exi'), 'omitnan');
            gy = median(yL - col(foll{i}, 'eyi'), 'omitnan');
            plot(ax, yL(m), xL(m), '-', 'Color', [cmap(i, :) 0.35], 'HandleVisibility', 'off');
            plot(ax, gy, gx, 'kx', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');
            scatter(ax, yL(end), xL(end), 50, cmap(i, :), 'filled', ...
                    'DisplayName', sprintf('%s final', foll{i}));
        end
        plot(ax, 0, 0, 'k^', 'MarkerFaceColor', 'k', 'DisplayName', 'leader');
        plot(ax, nan, nan, 'kx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'slot G_i');
        xline(ax, 0, 'k-', 'HandleVisibility', 'off'); yline(ax, 0, 'k-', 'HandleVisibility', 'off');
        xlabel(ax, 'y_L  right (m)'); ylabel(ax, 'x_L  forward (m)');
        title(ax, 'Positions in leader frame, t \geq 0 (paper Fig 5c)');
        axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');
        legend(ax, 'Location', 'southoutside', 'NumColumns', 3);

        % =======================================================================
        % FIGURE 2 -- Leader convergence
        % =======================================================================
        fig = figure('Name', '2 Leader convergence', 'Color', 'w');  figs(2) = fig;
        tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        if exactMode
            title(tl, 'Leader convergence (exact mode: leader generated on the path)');
        else
            title(tl, sprintf('Leader convergence onto the path from d_0 = %+.1f m (%s)', ...
                  dL(iOn), offTxt));
        end
        if pathMismatch
            title(tl, sprintf(['WRONG REFERENCE PATH? Leader stays %.1f m from the assumed path ', ...
                  '(%s). Check run_config.txt or pass ''Offset'',[N E].'], max(late), offTxt), ...
                  'Color', [0.8 0 0]);
        end
        axs = gobjects(3, 1);

        axs(1) = nexttile(tl); hold(axs(1), 'on');
        if o.LeaderDLog
            res = csv_resolution(csvFile);
            plot(axs(1), t, max(abs(dL), res), 'k-', 'LineWidth', 1.4, 'DisplayName', '|d|');
            if ~isnan(L.tau)
                plot(axs(1), fitT, abs(fitD), 'r--', 'LineWidth', 1.4, 'DisplayName', ...
                     sprintf('fit d_0 e^{-t/\\tau}, \\tau = %.2f s', L.tau));
            end
            set(axs(1), 'YScale', 'log');
            yline(axs(1), o.LeaderDTol, 'r:', sprintf('%.2f m', o.LeaderDTol), 'HandleVisibility', 'off');
            yline(axs(1), res, ':', sprintf('CSV resolution %.0e m', res), ...
                  'Color', [.4 .4 .4], 'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
            ylabel(axs(1), '|d|  cross-track (m)');
        else
            plot(axs(1), t, dL, 'k-', 'LineWidth', 1.6, 'DisplayName', 'd (leader)');
            if ~isnan(L.tau)
                plot(axs(1), fitT, fitD, 'r--', 'LineWidth', 1.6, 'DisplayName', ...
                     sprintf('fit d_0 e^{-t/\\tau}:  \\tau = %.2f s  (theory %.2f s)', ...
                             L.tau, L.tau_theory));
            end
            yline(axs(1), 0, 'k-', 'HandleVisibility', 'off');
            yline(axs(1),  o.LeaderDTol, 'r:', 'HandleVisibility', 'off');
            yline(axs(1), -o.LeaderDTol, 'r:', 'HandleVisibility', 'off');
            ylabel(axs(1), 'd  cross-track (m)');
        end
        if ~isnan(L.settle_ht) && L.settle_ht > 0
            xline(axs(1), L.settle_ht, '--', 'Color', blue, 'LineWidth', 1.2, ...
                  'DisplayName', sprintf('\\chi_l - \\chi_d settled (%.1f s): exponential phase', ...
                                         L.settle_ht));
        end
        mark(axs(1), L.settle_d, 'k');
        legend(axs(1), 'Location', 'best');
        if exactMode
            % state the scale in the title: the residual is ~1e-3 m and MATLAB puts
            % that factor in a small "x10^-3" corner label that is easy to misread as m
            title(axs(1), sprintf(['Cross-track d: exact mode, max |d| = %.1e m ', ...
                  '(on the path by construction; note the axis exponent)'], ...
                  max(abs(dL(t >= 0)))));
        elseif isnan(L.tau)
            title(axs(1), sprintf('Cross-track d: |d| \\leq %.2f m reached %s, settled %s', ...
                  o.LeaderDTol, fmt_t(L.reach_d), fmt_t(L.settle_d)));
        else
            title(axs(1), sprintf(['Cross-track d: exponential decay, \\tau = %.2f s ', ...
                  '(theory %.2f s); |d| \\leq %.2f m at %s'], ...
                  L.tau, L.tau_theory, o.LeaderDTol, fmt_t(L.settle_d)));
        end

        axs(2) = nexttile(tl); hold(axs(2), 'on');
        plot(axs(2), t, ePath, 'k-', 'LineWidth', 1.4, ...
             'DisplayName', sprintf('\\chi_l - \\chi_p  vs path tangent (settles %s)', fmt_t(L.settle_hp)));
        plot(axs(2), t, eTilde, '--', 'Color', blue, 'LineWidth', 1.4, ...
             'DisplayName', sprintf('\\chi_l - \\chi_d  vector-field error (settles %s)', fmt_t(L.settle_ht)));
        yline(axs(2),  o.LeaderChiTol, 'r--', 'HandleVisibility', 'off');
        yline(axs(2), -o.LeaderChiTol, 'r--', 'HandleVisibility', 'off');
        mark(axs(2), L.settle_hp, 'k'); mark(axs(2), L.settle_ht, blue);
        ylabel(axs(2), 'heading error (rad)');
        if exactMode
            title(axs(2), sprintf(['Leader heading error (band \\pm%.2f rad): exact mode, ', ...
                  'max |\\chi_l - \\chi_p| = %.1e rad, max |\\chi_l - \\chi_d| = %.1e rad'], ...
                  o.LeaderChiTol, max(abs(ePath(t >= 0))), max(abs(eTilde(t >= 0)))));
        else
            title(axs(2), sprintf('Leader heading error (band \\pm%.2f rad)', o.LeaderChiTol));
        end
        legend(axs(2), 'Location', 'best');

        axs(3) = nexttile(tl);
        yyaxis(axs(3), 'left');  plot(axs(3), t, T.leader_speed, 'LineWidth', 1.2);
        ylabel(axs(3), 'speed (m/s)');
        ylim(axs(3), [0, 1.2 * max(T.leader_speed)]);     % keep the cruise line off the frame
        yyaxis(axs(3), 'right'); plot(axs(3), t, T.leader_chidot, 'LineWidth', 1.2);
        ylabel(axs(3), 'turn rate (rad/s)');
        title(axs(3), 'Leader speed and turn rate');
        xlabel(axs(3), 'time since leader start (s)');
        finish(axs, t);
        % zoom on the convergence: the leader is on the path long before the log ends
        if ~exactMode && ~isnan(L.settle_d) && L.settle_d > 0
            xlim(axs(1), [t(1), min(t(end), max(120, 2.5 * L.settle_d))]);
        end

        % =======================================================================
        % FIGURE 3 -- Follower convergence
        % =======================================================================
        fig = figure('Name', '3 Follower convergence', 'Color', 'w');  figs(3) = fig;
        tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        title(tl, sprintf('Follower convergence -- formation settled: distance %s, heading %s', ...
              fmt_t(formD), fmt_t(formC)));
        axs = gobjects(2, 1);

        axs(1) = nexttile(tl); hold(axs(1), 'on');
        for i = 1:nF
            plot(axs(1), t, col(foll{i}, 'Di'), 'LineWidth', 1.1, 'Color', cmap(i, :), ...
                 'DisplayName', sprintf('%s  settles %s', foll{i}, fmt_t(sD(i))));
        end
        set(axs(1), 'YScale', 'log');
        yline(axs(1), o.DTol, 'k--', sprintf('%.2f m', o.DTol), 'HandleVisibility', 'off');
        for i = 1:nF; mark(axs(1), sD(i), cmap(i, :)); end
        ylabel(axs(1), 'D_i (m)');
        title(axs(1), sprintf('Formation error D_i, band %.2f m (paper Fig 5b)', o.DTol));
        legend(axs(1), 'Location', 'northeast');

        axs(2) = nexttile(tl); hold(axs(2), 'on');
        for i = 1:nF
            % old residual (chi_err_res / chi_err column), kept for comparison
            plot(axs(2), t, col(foll{i}, chiSuf), '--', 'LineWidth', 0.5, ...
                 'Color', [cmap(i, :) 0.6], 'DisplayName', ...
                 sprintf('%s  old %s  (settles %s)', foll{i}, chiLbl(1:end-6), fmt_t(sCold(i))));
            plot(axs(2), t, chiT(:, i), '-', 'LineWidth', 1.1, 'Color', cmap(i, :), ...
                 'DisplayName', sprintf('%s  \\chi_i - \\chi_d  (settles %s)', foll{i}, fmt_t(sC(i))));
        end
        yline(axs(2),  o.ChiTol, 'k--', 'HandleVisibility', 'off');
        yline(axs(2), -o.ChiTol, 'k--', 'HandleVisibility', 'off');
        for i = 1:nF; mark(axs(2), sC(i), cmap(i, :)); end
        ylabel(axs(2), '\chi_i - \chi_d (rad)');
        title(axs(2), sprintf(['Heading error \\chi_i - \\chi_d, \\chi_d = \\chi_l + \\Delta_i ', ...
              '- \\chi_\\infty(2/\\pi)atan(k_y e_y) (solid; rebuilt, k_y = %.2f) vs old ', ...
              '\\chi_i - \\chi_l - \\Delta_i (thin dashed); band \\pm%.2f rad'], o.Ky, o.ChiTol));
        xlabel(axs(2), 'time since leader start (s)');
        legend(axs(2), 'Location', 'northeast', 'NumColumns', 2);
        finish(axs, t);

        % =======================================================================
        % FIGURE 4 -- Follower motion (speed, turn rate, heading)
        % =======================================================================
        fig = figure('Name', '4 Follower motion', 'Color', 'w');  figs(4) = fig;
        tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        title(tl, 'Follower motion');
        axs = gobjects(3, 1);

        axs(1) = nexttile(tl); hold(axs(1), 'on');
        plot(axs(1), t, T.leader_speed, 'k--', 'LineWidth', 1.5, 'DisplayName', 'leader');
        for i = 1:nF
            plot(axs(1), t, col(foll{i}, 'speed'), 'LineWidth', 1.0, 'Color', cmap(i, :), ...
                 'DisplayName', foll{i});
        end
        ylabel(axs(1), 'ground speed (m/s)'); title(axs(1), 'Speed (paper Fig 4c)');
        legend(axs(1), 'Location', 'eastoutside');

        axs(2) = nexttile(tl); hold(axs(2), 'on');
        for i = 1:nF
            plot(axs(2), t, col(foll{i}, 'chidot'), 'LineWidth', 0.8, 'Color', cmap(i, :), ...
                 'DisplayName', foll{i});
        end
        ylabel(axs(2), 'yaw rate (rad/s)');
        title(axs(2), 'Measured turn rate (paper Fig 4b)');
        legend(axs(2), 'Location', 'eastoutside');

        axs(3) = nexttile(tl); hold(axs(3), 'on');
        plot(axs(3), t, unwrap(T.leader_chi), 'k-', 'LineWidth', 2, 'DisplayName', 'leader');
        for i = 1:nF
            plot(axs(3), t, unwrap(col(foll{i}, 'chi')), 'LineWidth', 1.0, 'Color', cmap(i, :), ...
                 'DisplayName', foll{i});
        end
        ylabel(axs(3), 'heading \chi (rad, unwrapped)');
        title(axs(3), 'Heading (paper Fig 5a)');
        xlabel(axs(3), 'time since leader start (s)');
        legend(axs(3), 'Location', 'eastoutside');
        finish(axs, t);

        % =======================================================================
        % FIGURE 5 -- Convergence type: asymptotic leader vs finite-time followers
        % =======================================================================
        % Followers use chiT (chi_i - chi_d, built above from <f>_chi_err_res and
        % <f>_eyi). eta and p are not logged: 'Eta' / 'P' default to the
        % follower_controller.py constants self.eta and self.n/self.m.
        try
            lg = @(x) log10(max(abs(x), res));
            fig = figure('Name', '5 Convergence type', 'Color', 'w');  figs(5) = fig;
            tl = tiledlayout(fig, 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
            title(tl, sprintf(['Convergence type -- leader: saturation law (asymptotic); ', ...
                  'followers: sign|\\chi|^p reaching law (finite time, \\eta = %.3f, p = %.2f)'], ...
                  o.Eta, o.P));
            axs = gobjects(2, 1);

            % leader: chi_l - chi_d_l (eTilde, Figure 2) and |d| (dL) with the
            % exponential fit, which is a straight line on a log axis
            axs(1) = nexttile(tl, [2 1]); hold(axs(1), 'on');
            plot(axs(1), t, lg(eTilde), '-', 'Color', blue, 'LineWidth', 1.4, ...
                 'DisplayName', '\chi_l - \chi_{d,l}  (rad)');
            plot(axs(1), t, lg(dL), 'k-', 'LineWidth', 1.4, 'DisplayName', 'd  (m)');
            if ~isnan(L.tau)
                plot(axs(1), fitT, lg(fitD), 'r--', 'LineWidth', 1.6, 'DisplayName', ...
                     sprintf('fit d_0 e^{-t/\\tau}, \\tau = %.2f s (theory %.2f s)', ...
                             L.tau, L.tau_theory));
            end
            yline(axs(1), log10(o.LeaderChiTol), '--', sprintf('%.2f rad', o.LeaderChiTol), ...
                  'Color', blue, 'HandleVisibility', 'off');
            yline(axs(1), log10(o.LeaderDTol), 'k--', sprintf('%.2f m', o.LeaderDTol), ...
                  'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
            yline(axs(1), log10(res), ':', sprintf('CSV resolution %.0e', res), 'Color', [.4 .4 .4], ...
                  'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');
            mark(axs(1), L.settle_ht, blue);
            ylabel(axs(1), 'log_{10}|error|');
            if exactMode
                title(axs(1), ['Leader (EXACT MODE: generated on the path, not steered -- no ', ...
                      'reaching law here, only the integration/rounding floor)']);
                xlim(axs(1), [-2, o.ZoomT]);
            else
                title(axs(1), sprintf(['Leader: \\chi_l - \\chi_{d,l} linear while |\\chi| > ', ...
                      '\\epsilon, then exponential; d exponential, \\tau = %.2f s'], L.tau));
                tEnd = o.ZoomT;
                if ~isnan(L.settle_d); tEnd = max(tEnd, 2 * L.settle_d); end
                xlim(axs(1), [-2, min(t(end), tEnd)]);
            end
            ylim(axs(1), [log10(res) - 0.3, max(1, log10(max(abs(dL))) + 0.5)]);
            legend(axs(1), 'Location', 'northeast');

            % followers: chi_i - chi_d, first ZoomT s, with the ideal finite-time
            % solution |chi(t)| = (|chi0|^(1-p) - eta (1-p) t)^(1/(1-p)) dashed
            axs(2) = nexttile(tl, [2 1]); hold(axs(2), 'on');
            for i = 1:nF
                plot(axs(2), t, lg(chiT(:, i)), 'LineWidth', 1.1, 'Color', cmap(i, :), ...
                     'DisplayName', sprintf('%s  t_s = %.2f s, t_{enter} = %s', ...
                                            foll{i}, tS(i), fmt_t2(tEW(i))));
                ti = linspace(0, tS(i), 200).';
                plot(axs(2), ti, lg(max(0, abs(c0(i))^pE - o.Eta * pE * ti).^(1/pE)), '--', ...
                     'Color', cmap(i, :), 'LineWidth', 0.9, 'HandleVisibility', 'off');
                mark(axs(2), tEW(i), cmap(i, :));
            end
            plot(axs(2), nan, nan, 'k--', 'DisplayName', 'ideal law (reaches 0 at t_s)');
            yline(axs(2), log10(bandW), 'k--', sprintf('band %.2f rad', bandW), 'HandleVisibility', 'off');
            yline(axs(2), log10(bandT), 'k:', sprintf('%g x ss RMS = %.3f rad', o.TightK, bandT), ...
                  'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
            ylabel(axs(2), 'log_{10}|\chi_i - \chi_d|');
            dTxt = '\Delta_i';
            if ~useDelta; dTxt = 'no \Delta (enable\_delta=false)'; end
            title(axs(2), sprintf(['Followers: \\chi_d = \\chi_l + %s - \\chi_\\infty(2/\\pi)', ...
                  'atan(k_y e_y), k_y = %.2f (rebuilt; \\chi_d not logged)'], dTxt, o.Ky));
            xlabel(axs(2), 'time since leader start (s)');
            xlim(axs(2), [-2, o.ZoomT]);
            ylim(axs(2), [max(log10(res), floor(log10(bandT)) - 1), 0.5]);
            legend(axs(2), 'Location', 'northeast');
            for k = 1:2
                grid(axs(k), 'on'); box(axs(k), 'on');
                xline(axs(k), 0, 'k:', 'HandleVisibility', 'off');
            end

            % the t_s table, printed in the figure
            axT = nexttile(tl); axis(axT, 'off');
            text(axT, 0, 1, [{sprintf(['t_s = |chi0|^(1-p)/(eta(1-p)); t_band = ideal time into ', ...
                 '%.2f rad; t_enter = first time in the band and staying to the end of the log'], ...
                 bandW)}; {tblHdr}; tblRows], 'Units', 'normalized', ...
                 'VerticalAlignment', 'top', 'FontName', 'FixedWidth', 'FontSize', 9, ...
                 'Interpreter', 'none');
        catch err
            warning('Figure 5 (Convergence type) failed; Figures 1-4 are unaffected:\n%s', ...
                    getReport(err, 'extended', 'hyperlinks', 'off'));
        end
    end

    % =======================================================================
    % Console summary
    % =======================================================================
    fprintf('\n=== Leader  (t = 0 at leader motion onset) ===\n');
    fprintf('d at t = 0: %+.3f m   |d| over last 20%%: max %.1e m\n', ...
            dL(iOn), max(abs(dL(end-nSS+1:end))));
    if ~isnan(L.tau)
        fprintf(['exponential decay: tau = %.2f s (fit over t = %.1f..%.1f s), ', ...
                 'theory pi/(2*chi_inf*v*K) = %.2f s\n'], ...
                L.tau, L.tau_window(1), L.tau_window(2), L.tau_theory);
    end
    fprintf('%-38s %11s %11s\n', 'error', 'reach', 'settle');
    fprintf('%-38s %11s %11s\n', sprintf('|d| <= %.2f m', o.LeaderDTol), ...
            fmt_t(L.reach_d), fmt_t(L.settle_d));
    fprintf('%-38s %11s %11s\n', sprintf('|chi_l - chi_p| <= %.2f rad (path)', o.LeaderChiTol), ...
            fmt_t(L.reach_hp), fmt_t(L.settle_hp));
    fprintf('%-38s %11s %11s\n', sprintf('|chi_l - chi_d| <= %.2f rad (VF)', o.LeaderChiTol), ...
            fmt_t(L.reach_ht), fmt_t(L.settle_ht));

    fprintf(['\n=== Followers  (D_i <= %.2f m, |chi_i - chi_d| <= %.2f rad; ', ...
             'ss_old / settle_old = old %s) ===\n'], o.DTol, o.ChiTol, chiSuf);
    fprintf('%6s %9s %9s %9s %9s %9s | %10s %10s %11s %11s %11s\n', 'drone', 'ss_Di', ...
            'rms_Di', 'peak_Di', 'ss_chi', 'ss_old', 'reach_D', 'settle_D', 'reach_chi', ...
            'settle_chi', 'settle_old');
    for i = 1:nF
        Di = col(foll{i}, 'Di'); ce = abs(chiT(:, i)); co = abs(col(foll{i}, chiSuf));
        fprintf('%6s %9.3f %9.3f %9.3f %9.4f %9.4f | %10s %10s %11s %11s %11s\n', foll{i}, ...
                mean(Di(end-nSS+1:end), 'omitnan'), sqrt(mean(Di(m).^2, 'omitnan')), ...
                max(Di(m)), mean(ce(end-nSS+1:end), 'omitnan'), ...
                mean(co(end-nSS+1:end), 'omitnan'), ...
                fmt_t(rD(i)), fmt_t(sD(i)), fmt_t(rC(i)), fmt_t(sC(i)), fmt_t(sCold(i)));
    end
    fprintf('Formation settled (slowest drone): distance %s, heading %s\n\n', ...
            fmt_t(formD), fmt_t(formC));

    fprintf(['=== Finite-time check: chi_i - chi_d  (eta = %.4f, p = %.2f, k_y = %.2f, ', ...
             'Delta %s from %s) ===\n'], o.Eta, o.P, o.Ky, string(useDelta), srcDelta);
    fprintf(['t_s = |chi0|^(1-p)/(eta(1-p)) (ideal time to 0); t_band = ideal time into ', ...
             '%.2f rad; t_enter = first time in the band and staying to the end of the log ', ...
             '(tight band %g x ss RMS = %.3f rad drawn for reference)\n'], bandW, o.TightK, bandT);
    fprintf('%s\n', tblHdr);
    fprintf('%s\n', tblRows{:});
    fprintf('\n');

    % ---- return the numbers ------------------------------------------------
    out.path = struct('name', pathName, 'offset_north', offN, 'offset_east', offE, ...
                      'amplitude', o.Amplitude, 'wavelength', o.Wavelength, 'source', src);
    out.leader = table(["|d|"; "chi_l - chi_p"; "chi_l - chi_d"], ...
        [o.LeaderDTol; o.LeaderChiTol; o.LeaderChiTol], ...
        [L.reach_d; L.reach_hp; L.reach_ht], [L.settle_d; L.settle_hp; L.settle_ht], ...
        'VariableNames', {'error', 'tol', 'reach_s', 'settle_s'});
    out.leader_decay = struct('tau_s', L.tau, 'tau_theory_s', L.tau_theory, ...
                              'fit_window_s', L.tau_window);
    out.followers = table(string(foll(:)), rD, sD, rC, sC, sCold, 'VariableNames', ...
        {'drone', 'reach_D_s', 'settle_D_s', 'reach_chi_s', 'settle_chi_s', 'settle_chi_old_s'});
    out.formation = struct('settle_D_s', formD, 'settle_chi_s', formC);
    out.finite_time = table(string(foll(:)), c0, tS, tSW, tEW, ...
        'VariableNames', {'drone', 'chi_tilde0', 't_s_pred', 't_band_pred', 't_enter'});
    out.finite_time.Properties.UserData = struct('band', bandW, 'tight_band', bandT, ...
        'eta', o.Eta, 'p', o.P, 'k_y', o.Ky, 'delta', useDelta);
    out.series = table(t, dL, ePath, eTilde, 'VariableNames', ...
        {'t', 'leader_d', 'leader_chi_minus_chi_p', 'leader_chi_minus_chi_d'});
    out.series_followers = array2table([t chiT], 'VariableNames', ...
        [{'t'}, strcat(foll(:).', '_chi_minus_chi_d')]);

    % ---- save all figures in one downloadable file ---------------------------
    out.files = struct('png', '', 'pdf', '');
    if o.Export
        [out.files.png, out.files.pdf] = export_figures(figs, csvFile, char(o.ExportDir));
    end
end

% ===========================================================================
% helpers
% ===========================================================================
function [pngFile, pdfFile] = export_figures(figs, csvFile, outDir)
% All figures into ONE image (2-column grid PNG) and ONE multi-page PDF (one
% figure per page, vector), named after the CSV, next to it by default.
    [csvDir, base] = fileparts(csvFile);
    if isempty(outDir)
        outDir = csvDir;
        if isempty(outDir); outDir = pwd; end
    end
    if ~isfolder(outDir); mkdir(outDir); end
    pngFile = fullfile(outDir, [base '_analysis.png']);
    pdfFile = fullfile(outDir, [base '_analysis.pdf']);
    if isfile(pdfFile); delete(pdfFile); end         % Append would add to an old one

    imgs = {};
    nPdf = 0;
    for k = 1:numel(figs)
        f = figs(k);
        if ~isgraphics(f, 'figure')
            warning('Figure %d was not created; it is missing from the export.', k);
            continue
        end
        try
            f.Units = 'pixels';
            sz = [1400 900];                         % default size for every panel
            if isstruct(f.UserData) && isfield(f.UserData, 'ExportSize'); sz = f.UserData.ExportSize; end
            f.Position(3:4) = sz;
            drawnow;
            exportgraphics(f, pdfFile, 'ContentType', 'vector', 'Append', nPdf > 0);
            nPdf = nPdf + 1;
            tmp = [tempname '.png'];
            exportgraphics(f, tmp, 'Resolution', 120, 'BackgroundColor', 'white');
            I = imread(tmp); delete(tmp);
            if size(I, 3) == 1; I = repmat(I, 1, 1, 3); end
            imgs{end+1} = I; %#ok<AGROW>
        catch err
            warning('Export of figure %d ("%s") failed:\n%s', k, f.Name, ...
                    getReport(err, 'extended', 'hyperlinks', 'off'));
        end
    end
    if isempty(imgs); pngFile = ''; return; end

    % 2-column grid on a white canvas
    nC = 2; nR = ceil(numel(imgs) / nC);
    h = max(cellfun(@(I) size(I, 1), imgs));
    w = max(cellfun(@(I) size(I, 2), imgs));
    canvas = uint8(255 * ones(nR * h, nC * w, 3));
    for k = 1:numel(imgs)
        I = imgs{k};
        r0 = floor((k - 1) / nC) * h;
        c0 = mod(k - 1, nC) * w;
        canvas(r0 + (1:size(I, 1)), c0 + (1:size(I, 2)), :) = I;
    end
    imwrite(canvas, pngFile);
    fprintf(['Saved %d of %d figures:\n  %s  (one image, %dx%d grid)\n', ...
             '  %s  (one figure per page, vector)\n'], numel(imgs), numel(figs), ...
            pngFile, nR, nC, pdfFile);
end

function [name, offN, offE, src, cfg] = resolve_path(csvFile, o)
% Path type and North/East offset: arguments first, else run_config.txt
% beside the CSV, else defaults (sinusoid, no offset). cfg also returns the
% run's amplitude / wavelength / enable_delta when they were set
% ("<node default>" entries are left out, so the node defaults apply).
    cfg = struct();
    fn = fullfile(fileparts(csvFile), 'run_config.txt');
    if isfile(fn)
        txt = fileread(fn);
        tk = regexp(txt, 'leader=(\w+)', 'tokens', 'once');
        if ~isempty(tk); cfg.leader = tk{1}; end
        tk = regexp(txt, 'path_offset=([-+0-9.eE]+)', 'tokens', 'once');
        if ~isempty(tk); cfg.path_offset = str2double(tk{1}); end
        tk = regexp(txt, 'path_offset_north=([-+0-9.eE]+)', 'tokens', 'once');
        if ~isempty(tk); cfg.path_offset_north = str2double(tk{1}); end
        tk = regexp(txt, 'amplitude=([-+0-9.eE]+)', 'tokens', 'once');
        if ~isempty(tk); cfg.amplitude = str2double(tk{1}); end
        tk = regexp(txt, 'wavelength=([-+0-9.eE]+)', 'tokens', 'once');
        if ~isempty(tk); cfg.wavelength = str2double(tk{1}); end
        tk = regexp(txt, 'enable_delta=(\w+)', 'tokens', 'once');
        if ~isempty(tk); cfg.enable_delta = any(strcmpi(tk{1}, {'true', '1', 'on'})); end
    else
        warning(['run_config.txt not found next to the CSV (%s). Path/offset fall back ', ...
                 'to arguments/defaults -- pass ''Path'' and ''Offset'',[N E] if this run ', ...
                 'used --path-offset.'], fn);
    end

    name = char(o.Path); srcP = 'argument';
    if isempty(name)
        if isfield(cfg, 'leader'); name = cfg.leader; srcP = 'run_config.txt';
        else; name = 'sinusoid'; srcP = 'default (no run_config.txt found)'; end
    end
    if strcmp(name, 'vf'); name = 'sinusoid'; end      % pre-merge leader name

    % 'Offset' may be E (scalar) or [N E] like --path-offset N,E
    offN = NaN; offE = NaN;
    if numel(o.Offset) == 2
        offN = o.Offset(1); offE = o.Offset(2);
    elseif ~isnan(o.Offset)
        offE = o.Offset;
    end
    if ~isnan(o.OffsetNorth); offN = o.OffsetNorth; end

    srcO = 'argument';
    if isnan(offE)
        if isfield(cfg, 'path_offset'); offE = cfg.path_offset; srcO = 'run_config.txt';
        else; offE = 0; srcO = 'default 0'; end
    end
    if isnan(offN)
        if isfield(cfg, 'path_offset_north'); offN = cfg.path_offset_north;
        else; offN = 0; end
    end
    src = sprintf('path from %s, offset from %s', srcP, srcO);
end

function P = make_path(name, xOff, yOff, o)
% Same paths as path_leader.py: start at the origin heading North, then
% shifted by xOff North and yOff East.
    P = struct('name', name, 'xOff', xOff, 'yOff', yOff, 'theta', 0);
    switch name
        case 'sinusoid'
            P.A = o.Amplitude; P.k = 2*pi / o.Wavelength;
        case 'figure8'
            P.A = o.F8Length; P.B = o.F8Width;
            P.theta = -atan2(2*P.B, P.A);        % start tangent -> North
        case 'circle'
            P.R = o.Radius; P.g = 1;
            if o.Direction < 0; P.g = -1; end
        case 'straight_line'
        otherwise
            error('Unknown path "%s" (sinusoid | figure8 | circle | straight_line)', name);
    end
end

function [r, r1, r2] = path_eval(P, u)
% Point and first two derivatives w.r.t. the path parameter u, NED [x y].
    switch P.name
        case 'sinusoid'
            A = P.A; k = P.k;
            r  = [u,  A*(cos(k*u) - 1)];
            r1 = [1, -A*k*sin(k*u)];
            r2 = [0, -A*k^2*cos(k*u)];
        case 'figure8'
            A = P.A; B = P.B;
            r  = [ A*sin(u),    B*sin(2*u)];
            r1 = [ A*cos(u),  2*B*cos(2*u)];
            r2 = [-A*sin(u), -4*B*sin(2*u)];
        case 'circle'
            R = P.R; g = P.g;
            r  = [ R*sin(u), g*R*(1 - cos(u))];
            r1 = [ R*cos(u), g*R*sin(u)];
            r2 = [-R*sin(u), g*R*cos(u)];
        case 'straight_line'
            r = [u, 0]; r1 = [1, 0]; r2 = [0, 0];
    end
    if P.theta ~= 0
        c = cos(P.theta); s = sin(P.theta); Rm = [c -s; s c];
        r = (Rm * r.').'; r1 = (Rm * r1.').'; r2 = (Rm * r2.').';
    end
    r(1) = r(1) + P.xOff;
    r(2) = r(2) + P.yOff;
end

function u = closest_point(P, xq, yq, u)
% Newton on (r(u) - q) . r'(u) = 0, warm-started; step capped at 2 m of arc.
    for it = 1:8
        [r, r1, r2] = path_eval(P, u);
        g  = (r(1) - xq)*r1(1) + (r(2) - yq)*r1(2);
        gp = r1(1)^2 + r1(2)^2 + (r(1) - xq)*r2(1) + (r(2) - yq)*r2(2);
        if abs(gp) < 1e-9; break; end
        lim = 2 / max(norm(r1), 1e-9);
        u = u - max(-lim, min(lim, g / gp));
    end
end

function ref = path_samples(P, xl)
% Reference path points [x y] for plotting.
    switch P.name
        case {'sinusoid', 'straight_line'}
            % u is the un-shifted North coordinate on these paths
            us = linspace(min(-10, min(xl) - P.xOff - 10), max(xl) - P.xOff + 10, 800);
        otherwise
            us = linspace(0, 2*pi, 800);
    end
    ref = zeros(numel(us), 2);
    for k = 1:numel(us); ref(k, :) = path_eval(P, us(k)); end
end

function [tau, fitT, fitD, win] = exp_fit(t, d, tStart)
% Fit d ~ d0*exp(-t/tau) over t >= tStart (vector-field heading settled)
% where |d| > 1 mm (above the numerical floor). NaN if there is no decay.
    tau = NaN; fitT = []; fitD = []; win = [NaN NaN];
    if isnan(tStart); return; end
    m = t >= tStart & abs(d) > 1e-3;
    if nnz(m) < 20; return; end
    p = polyfit(t(m), log(abs(d(m))), 1);
    if p(1) >= 0; return; end
    tau = -1 / p(1);
    win = [t(find(m, 1, 'first')), t(find(m, 1, 'last'))];
    fitT = linspace(win(1), win(2), 300).';
    fitD = sign(median(d(m))) * exp(polyval(p, fitT));
end

function res = csv_resolution(csvFile)
% Rounding step of leader_x in the CSV (1e-4 m for old 4-decimal logs,
% 1e-6 m since the 6-decimal logger change).
    res = 1e-6;
    fid = fopen(csvFile, 'r');
    if fid < 0; return; end
    fgetl(fid);                                    % header
    line = fgetl(fid);
    fclose(fid);
    if ~ischar(line); return; end
    parts = strsplit(line, ',');
    if numel(parts) >= 2 && contains(parts{2}, '.')
        res = 10 ^ -numel(extractAfter(parts{2}, '.'));
    end
end

function [tReach, tSettle] = conv_time(t, e, tol)
% reach: first t >= 0 with |e| <= tol.  settle: first t >= 0 after which
% |e| stays <= tol to the end of the log (NaN samples count as outside).
    m  = t >= 0;
    tt = t(m);
    bad = abs(e(m)) > tol | isnan(e(m));
    kIn = find(~bad, 1, 'first');
    if isempty(kIn); tReach = NaN; else; tReach = tt(kIn); end
    kOut = find(bad, 1, 'last');
    if isempty(kOut)
        tSettle = 0;                 % never left the band
    elseif kOut == numel(tt)
        tSettle = NaN;               % still outside at the end of the log
    else
        tSettle = tt(kOut + 1);
    end
end

function s = slowest(x)
    if any(isnan(x)); s = NaN; else; s = max(x); end
end

function mark(ax, ts, c)
% Dotted vertical line at a settle time (skipped for 0 / not converged).
    if ~isnan(ts) && ts > 0
        xline(ax, ts, ':', 'Color', c, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end

function finish(axs, t)
% Common styling for stacked time plots: grid, t = 0 marker, then link the
% x axes and set the range once (setting it before linkaxes let a yyaxis
% panel reset the shared range).
    for k = 1:numel(axs)
        grid(axs(k), 'on'); box(axs(k), 'on');
        if k == 1
            xline(axs(k), 0, 'k:', 'leader starts', 'HandleVisibility', 'off', ...
                  'LabelVerticalAlignment', 'bottom');
        else
            xline(axs(k), 0, 'k:', 'HandleVisibility', 'off');
        end
    end
    linkaxes(axs, 'x');
    xlim(axs(1), [t(1) t(end)]);
end

function fig = report_fig(name, sz)
% Figure for the report: white, larger fonts, export size in UserData.
    fig = figure('Name', name, 'Color', 'w', 'Units', 'pixels');
    fig.Position(3:4) = sz;
    fig.UserData = struct('ExportSize', sz);
    set(fig, 'DefaultAxesFontSize', 12, 'DefaultLegendFontSize', 11, ...
             'DefaultAxesTitleFontWeight', 'normal', 'DefaultAxesTitleFontSizeMultiplier', 1.15);
end

function report_axes(axs, xl)
% Report styling for stacked time plots: grid, t = 0 marker, shared x range.
    for k = 1:numel(axs)
        grid(axs(k), 'on'); box(axs(k), 'on');
        if k == 1
            xline(axs(k), 0, 'k:', 'leader starts', 'LabelVerticalAlignment', 'bottom', ...
                  'HandleVisibility', 'off');
        else
            xline(axs(k), 0, 'k:', 'HandleVisibility', 'off');
        end
    end
    linkaxes(axs, 'x');
    xlim(axs(1), xl);
end

function w = wrapPi(a)
    w = mod(a + pi, 2*pi) - pi;
end

function s = fmt_t(x)
    if isnan(x); s = 'not conv.'; else; s = sprintf('%.1f s', x); end
end

function s = fmt_t2(x)
    if isnan(x); s = 'not conv.'; else; s = sprintf('%.2f s', x); end
end
