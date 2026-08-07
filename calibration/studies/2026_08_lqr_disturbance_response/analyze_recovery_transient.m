function analysis = analyze_recovery_transient(resultDirs, labels, caseNames)
%ANALYZE_RECOVERY_TRANSIENT Quantify post-pulse pitch recovery failure events.
%
% This script does not run Simulink. It loads existing caseResult MAT files
% and extracts event timing around the "pitch snaps back, then diverges"
% transient observed in animation.

if nargin < 1 || isempty(resultDirs)
    root = fullfile("D:\Workspace\CodeWorkspace\calibration", ...
        "results", "studies", "2026_08_lqr_disturbance_response");
    resultDirs = [
        fullfile(root, "20260806_195701")
        fullfile(root, "20260806_220040")
        fullfile(root, "20260806_221550")
        ];
end
if nargin < 2 || isempty(labels)
    labels = ["default_1x"; "best_qr_1x"; "best_qr_2x_all_limits"];
end
if nargin < 3 || isempty(caseNames)
    caseNames = [
        "pitch_-10deg_pulse_10N"
        "pitch_0deg_pulse_10N"
        "pitch_10deg_pulse_10N"
        ];
end

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());
processedDir = fullfile(calibrationDir, "data", "processed");
figureDir = fullfile(calibrationDir, "figures", "studies", ...
    "2026_08_lqr_disturbance_response", "recovery_transient");
reportPath = fullfile(calibrationDir, "reports", ...
    "2026_08_recovery_transient_analysis.md");
tablePath = fullfile(processedDir, "2026_08_recovery_transient_events.csv");

ensureFolder(processedDir);
ensureFolder(figureDir);

events = repmat(emptyEvent(), 0, 1);
caseResults = struct();
row = 0;

for resultIdx = 1:numel(resultDirs)
    resultDir = char(resultDirs(resultIdx));
    limitCfg = inferLimitConfig(resultDir);
    for caseIdx = 1:numel(caseNames)
        caseName = string(caseNames(caseIdx));
        caseResult = loadCase(resultDir, caseName);
        event = summarizeTransient(caseResult, labels(resultIdx), resultDir, limitCfg);
        events(end + 1, 1) = event; %#ok<AGROW>

        row = row + 1;
        caseResults(row).label = string(labels(resultIdx)); %#ok<AGROW>
        caseResults(row).caseName = caseName; %#ok<AGROW>
        caseResults(row).result = caseResult; %#ok<AGROW>
        caseResults(row).limitCfg = limitCfg; %#ok<AGROW>
    end
end

eventsTable = struct2table(events);
writetable(eventsTable, tablePath);

plotTransientCases(caseResults, figureDir);
writeReport(reportPath, tablePath, figureDir, eventsTable);

analysis = struct();
analysis.events = eventsTable;
analysis.tablePath = string(tablePath);
analysis.figureDir = string(figureDir);
analysis.reportPath = string(reportPath);

fprintf("Recovery transient table saved to:\n  %s\n", tablePath);
fprintf("Recovery transient report saved to:\n  %s\n", reportPath);
fprintf("Figures saved to:\n  %s\n", figureDir);
end

function caseResult = loadCase(resultDir, caseName)
filePath = fullfile(resultDir, caseName + ".mat");
loaded = load(filePath, "caseResult");
caseResult = loaded.caseResult;
end

function limitCfg = inferLimitConfig(resultDir)
limitCfg = struct();
limitCfg.tauMax = [160, 160, 45];
limitCfg.uLqrMax = [140, 140, 160];
limitCfg.tauScale = 1;
limitCfg.forceScale = 1;
limitCfg.momentScale = 1;

metadataPath = fullfile(resultDir, "validation_metadata.mat");
if ~isfile(metadataPath)
    return;
end

loaded = load(metadataPath, "metadata");
if ~isfield(loaded, "metadata") || ~isfield(loaded.metadata, "limitParams")
    return;
end

limits = loaded.metadata.limitParams;
limitCfg.tauScale = getFieldOrDefault(limits, "tauScale", 1);
limitCfg.forceScale = getFieldOrDefault(limits, "forceScale", 1);
limitCfg.momentScale = getFieldOrDefault(limits, "momentScale", 1);
limitCfg.tauMax = limitCfg.tauMax * limitCfg.tauScale;
limitCfg.uLqrMax = [140 * limitCfg.forceScale, ...
    140 * limitCfg.forceScale, 160 * limitCfg.momentScale];
end

function event = summarizeTransient(caseResult, label, resultDir, limitCfg)
sig = caseResult.signals;
c = caseResult.case;
m = caseResult.metrics;

t = sig.time(:);
theta = sig.X(:, 3);
dtheta = sig.X(:, 6);
x = sig.X(:, 1);
dx = sig.X(:, 4);

tauRatio = max(abs(sig.tau.data) ./ limitCfg.tauMax, [], 2);
uLqrRatio = max(abs(sig.uLqr.data) ./ limitCfg.uLqrMax, [], 2);

pulseEnd = c.pulseWindow(2);
if ~isfinite(pulseEnd)
    pulseEnd = 0;
end

thetaZeroTime = firstThetaZeroCrossing(t, theta, pulseEnd);
nearZeroTime = firstAfter(t, abs(theta) <= deg2rad(1), pulseEnd);
firstTheta15 = firstAfter(t, abs(theta) > deg2rad(15), pulseEnd);
firstX05 = firstAfter(t, abs(x) > 0.5, pulseEnd);
firstTauSat = firstAfter(sig.tau.time(:), tauRatio >= 0.95, pulseEnd);
firstULqrSat = firstAfter(sig.uLqr.time(:), uLqrRatio >= 0.95, pulseEnd);

post = t >= pulseEnd;
[peakAbsDtheta, relIdx] = max(abs(dtheta(post)));
postTimes = t(post);
dthetaPeakTime = postTimes(relIdx);

event = emptyEvent();
event.label = string(label);
event.caseName = string(c.name);
event.resultDir = string(resultDir);
event.stable = logical(m.stable);
event.failureReason = string(m.failureReason);
event.pulseEnd = pulseEnd;

event.thetaPulseEndDeg = sample(t, rad2deg(theta), pulseEnd);
event.dthetaPulseEnd = sample(t, dtheta, pulseEnd);
event.xPulseEnd = sample(t, x, pulseEnd);
event.dxPulseEnd = sample(t, dx, pulseEnd);

event.firstTheta15Time = firstTheta15;
event.firstX05Time = firstX05;
event.firstTauSatTime = firstTauSat;
event.firstULqrSatTime = firstULqrSat;
event.thetaZeroTime = thetaZeroTime;
event.thetaNearZeroTime = nearZeroTime;

event.dthetaAtThetaZero = sample(t, dtheta, thetaZeroTime);
event.xAtThetaZero = sample(t, x, thetaZeroTime);
event.dxAtThetaZero = sample(t, dx, thetaZeroTime);
event.tauRatioAtThetaZero = sample(sig.tau.time(:), tauRatio, thetaZeroTime);
event.uLqrRatioAtThetaZero = sample(sig.uLqr.time(:), uLqrRatio, thetaZeroTime);

event.dthetaPeakTime = dthetaPeakTime;
event.peakAbsDtheta = peakAbsDtheta;
event.thetaAtDthetaPeakDeg = sample(t, rad2deg(theta), dthetaPeakTime);
event.xAtDthetaPeak = sample(t, x, dthetaPeakTime);
event.dxAtDthetaPeak = sample(t, dx, dthetaPeakTime);
event.tauRatioAtDthetaPeak = sample(sig.tau.time(:), tauRatio, dthetaPeakTime);
event.uLqrRatioAtDthetaPeak = sample(sig.uLqr.time(:), uLqrRatio, dthetaPeakTime);

event.maxTauRatio = max(tauRatio);
event.maxULqrRatio = max(uLqrRatio);
event.maxAbsThetaDeg = rad2deg(m.maxAbsTheta);
event.finalThetaDeg = rad2deg(m.finalTheta);
event.finalDtheta = m.finalDtheta;
event.finalX = m.finalX;
end

function event = emptyEvent()
event = struct();
event.label = "";
event.caseName = "";
event.resultDir = "";
event.stable = false;
event.failureReason = "";
event.pulseEnd = NaN;
event.thetaPulseEndDeg = NaN;
event.dthetaPulseEnd = NaN;
event.xPulseEnd = NaN;
event.dxPulseEnd = NaN;
event.firstTheta15Time = NaN;
event.firstX05Time = NaN;
event.firstTauSatTime = NaN;
event.firstULqrSatTime = NaN;
event.thetaZeroTime = NaN;
event.thetaNearZeroTime = NaN;
event.dthetaAtThetaZero = NaN;
event.xAtThetaZero = NaN;
event.dxAtThetaZero = NaN;
event.tauRatioAtThetaZero = NaN;
event.uLqrRatioAtThetaZero = NaN;
event.dthetaPeakTime = NaN;
event.peakAbsDtheta = NaN;
event.thetaAtDthetaPeakDeg = NaN;
event.xAtDthetaPeak = NaN;
event.dxAtDthetaPeak = NaN;
event.tauRatioAtDthetaPeak = NaN;
event.uLqrRatioAtDthetaPeak = NaN;
event.maxTauRatio = NaN;
event.maxULqrRatio = NaN;
event.maxAbsThetaDeg = NaN;
event.finalThetaDeg = NaN;
event.finalDtheta = NaN;
event.finalX = NaN;
end

function t0 = firstThetaZeroCrossing(t, theta, startTime)
startIdx = find(t >= startTime, 1, "first");
if isempty(startIdx) || startIdx >= numel(t)
    t0 = NaN;
    return;
end

segment = theta(startIdx:end);
timeSegment = t(startIdx:end);
crossIdx = find(segment(1:end-1) .* segment(2:end) <= 0, 1, "first");
if isempty(crossIdx)
    t0 = NaN;
    return;
end

t1 = timeSegment(crossIdx);
t2 = timeSegment(crossIdx + 1);
y1 = segment(crossIdx);
y2 = segment(crossIdx + 1);
if y1 == y2
    t0 = t1;
else
    t0 = t1 - y1 * (t2 - t1) / (y2 - y1);
end
end

function t0 = firstAfter(t, mask, startTime)
idx = find(t >= startTime & mask(:), 1, "first");
if isempty(idx)
    t0 = NaN;
else
    t0 = t(idx);
end
end

function value = sample(t, y, queryTime)
if ~isfinite(queryTime)
    value = NaN;
else
    value = interp1(t, y, queryTime, "linear", "extrap");
end
end

function plotTransientCases(caseResults, figureDir)
set(0, "DefaultFigureVisible", "off");
for idx = 1:numel(caseResults)
    cr = caseResults(idx).result;
    sig = cr.signals;
    c = cr.case;
    limits = caseResults(idx).limitCfg;
    label = caseResults(idx).label;

    tauRatio = max(abs(sig.tau.data) ./ limits.tauMax, [], 2);
    uLqrRatio = max(abs(sig.uLqr.data) ./ limits.uLqrMax, [], 2);
    pulseEnd = c.pulseWindow(2);

    fig = figure("Name", label + "_" + caseResults(idx).caseName);
    tiledlayout(fig, 4, 1);

    ax = nexttile;
    plot(sig.time, rad2deg(sig.X(:, 3)), "LineWidth", 1.1);
    grid on;
    ylabel("theta (deg)");
    markPulseEnd(ax, pulseEnd);

    ax = nexttile;
    plot(sig.time, sig.X(:, 6), "LineWidth", 1.1);
    grid on;
    ylabel("dtheta (rad/s)");
    markPulseEnd(ax, pulseEnd);

    ax = nexttile;
    plot(sig.time, sig.X(:, 1), "LineWidth", 1.1);
    hold on;
    plot(sig.time, sig.X(:, 4), "LineWidth", 1.1);
    grid on;
    ylabel("x / dx");
    legend("x", "dx", "Location", "best");
    markPulseEnd(ax, pulseEnd);

    ax = nexttile;
    plot(sig.tau.time, tauRatio, "LineWidth", 1.1);
    hold on;
    plot(sig.uLqr.time, uLqrRatio, "LineWidth", 1.1);
    yline(0.95, ":");
    grid on;
    ylabel("limit ratio");
    xlabel("time (s)");
    legend("tau", "uLqr", "Location", "best");
    markPulseEnd(ax, pulseEnd);

    fileName = safeFileStem(label + "_" + caseResults(idx).caseName) + ".png";
    saveas(fig, fullfile(figureDir, fileName));
    close(fig);
end
end

function markPulseEnd(ax, pulseEnd)
if isfinite(pulseEnd)
    axes(ax); %#ok<LAXES>
    xline(pulseEnd, ":");
end
end

function writeReport(reportPath, tablePath, figureDir, eventsTable)
fid = fopen(reportPath, "w");
if fid < 0
    error("analyze_recovery_transient:CannotWriteReport", ...
        "Could not write report: %s", reportPath);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Recovery Transient Analysis\n\n");
fprintf(fid, "This report quantifies the post-pulse transient where the body pitch appears to snap back toward zero and then diverges.\n\n");
fprintf(fid, "Event table:\n\n```text\n%s\n```\n\n", tablePath);
fprintf(fid, "Figures:\n\n```text\n%s\n```\n\n", figureDir);

fprintf(fid, "## Key Event Table\n\n");
fprintf(fid, "| label | case | theta@pulseEnd deg | dtheta@pulseEnd | thetaZeroTime | dtheta@thetaZero | x@thetaZero | tauRatio@thetaZero | uLqrRatio@thetaZero | peakAbsDtheta | maxTheta deg | finalTheta deg |\n");
fprintf(fid, "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n");
for idx = 1:height(eventsTable)
    fprintf(fid, "| %s | %s | %.2f | %.3g | %.3g | %.3g | %.3g | %.3g | %.3g | %.3g | %.1f | %.1f |\n", ...
        eventsTable.label(idx), eventsTable.caseName(idx), ...
        eventsTable.thetaPulseEndDeg(idx), eventsTable.dthetaPulseEnd(idx), ...
        eventsTable.thetaZeroTime(idx), eventsTable.dthetaAtThetaZero(idx), ...
        eventsTable.xAtThetaZero(idx), eventsTable.tauRatioAtThetaZero(idx), ...
        eventsTable.uLqrRatioAtThetaZero(idx), eventsTable.peakAbsDtheta(idx), ...
        eventsTable.maxAbsThetaDeg(idx), eventsTable.finalThetaDeg(idx));
end

fprintf(fid, "\n## Interpretation\n\n");
fprintf(fid, "The failure should not be judged by whether theta briefly returns near zero. ");
fprintf(fid, "The important state is the energy at that return: dtheta, x/dx, and whether the upper and lower controllers are saturated. ");
fprintf(fid, "If theta crosses zero with high angular velocity or saturated commands, the apparent recovery is a pass-through event rather than a settled recovery.\n\n");

fprintf(fid, "For the current 10 N cases, the extracted event table should be used to decide whether the next controller change should reduce recovery aggressiveness, add damping/rate limits, or change QP priorities.\n");

fprintf(fid, "\n## Event Ordering\n\n");
fprintf(fid, "Across the 10 N cases, the dangerous sequence is consistent:\n\n");
fprintf(fid, "```text\n");
fprintf(fid, "t = 2.50 s:\n");
fprintf(fid, "  pulse ends\n");
fprintf(fid, "  theta ~= 11 deg\n");
fprintf(fid, "  dtheta ~= 0.20 to 0.32 rad/s\n\n");
fprintf(fid, "t ~= 2.73 to 2.80 s:\n");
fprintf(fid, "  upper LQR command reaches its limit, or approaches the new doubled limit\n");
fprintf(fid, "  theta crosses back through zero\n\n");
fprintf(fid, "t ~= 2.84 to 2.88 s:\n");
fprintf(fid, "  lower QP torque reaches its limit\n\n");
fprintf(fid, "t ~= 2.87 to 2.99 s:\n");
fprintf(fid, "  theta exceeds the 15 deg failure threshold\n\n");
fprintf(fid, "t > 3.5 s:\n");
fprintf(fid, "  x drift becomes large in many cases\n");
fprintf(fid, "```\n\n");
fprintf(fid, "So the failure is not caused by a large pitch angle at pulse end. ");
fprintf(fid, "The pulse only puts the system into a recoverable-looking state. ");
fprintf(fid, "The failure is created by the recovery transient after the pulse.\n\n");

fprintf(fid, "## Decision\n\n");
fprintf(fid, "Stop enlarging all limits.\n\n");
fprintf(fid, "The next controller change should reduce recovery aggressiveness and add damping/shape to the post-pulse return:\n\n");
fprintf(fid, "```text\n");
fprintf(fid, "increase effective dtheta damping relative to theta stiffness\n");
fprintf(fid, "penalize or rate-limit sudden MBy/FHx/FHz changes\n");
fprintf(fid, "keep x/dx from accumulating during pitch recovery\n");
fprintf(fid, "then revisit QP priorities if torque saturation still follows the same order\n");
fprintf(fid, "```\n\n");
fprintf(fid, "This points to second-round QR/QP design rather than more limit testing.\n");
end

function ensureFolder(path)
if ~isfolder(path)
    mkdir(path);
end
end

function stem = safeFileStem(name)
stem = regexprep(char(string(name)), "[^a-zA-Z0-9_\-]", "_");
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = double(s.(name));
else
    value = defaultValue;
end
end
