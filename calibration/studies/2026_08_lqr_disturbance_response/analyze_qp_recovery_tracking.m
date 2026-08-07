function analysis = analyze_qp_recovery_tracking(resultDirs, labels, caseNames)
%ANALYZE_QP_RECOVERY_TRACKING Inspect MBy_des to tau_h tracking in recovery.
%
% This script does not run Simulink. It loads existing caseResult MAT files
% and checks whether the lower QP realizes the upper-layer pitch moment target
% during the post-pulse recovery transient.

if nargin < 1 || isempty(resultDirs)
    root = fullfile("D:\Workspace\CodeWorkspace\calibration", ...
        "results", "studies", "2026_08_lqr_disturbance_response");
    resultDirs = [
        fullfile(root, "20260806_195701")
        fullfile(root, "20260806_220040")
        fullfile(root, "20260807_182335")
        ];
end
if nargin < 2 || isempty(labels)
    labels = ["default_1x"; "round1_best"; "round2_best"];
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
    "2026_08_lqr_disturbance_response", "qp_recovery_tracking");
reportPath = fullfile(calibrationDir, "reports", ...
    "2026_08_qp_recovery_tracking.md");
tablePath = fullfile(processedDir, "2026_08_qp_recovery_tracking.csv");

ensureFolder(processedDir);
ensureFolder(figureDir);

rows = repmat(emptyRow(), 0, 1);
caseResults = struct();
caseCounter = 0;

for resultIdx = 1:numel(resultDirs)
    resultDir = char(resultDirs(resultIdx));
    for caseIdx = 1:numel(caseNames)
        caseName = string(caseNames(caseIdx));
        caseResult = loadCase(resultDir, caseName);
        row = summarizeCase(caseResult, labels(resultIdx), resultDir);
        rows(end + 1, 1) = row; %#ok<AGROW>

        caseCounter = caseCounter + 1;
        caseResults(caseCounter).label = string(labels(resultIdx)); %#ok<AGROW>
        caseResults(caseCounter).caseName = caseName; %#ok<AGROW>
        caseResults(caseCounter).result = caseResult; %#ok<AGROW>
    end
end

trackingTable = struct2table(rows);
writetable(trackingTable, tablePath);

plotCases(caseResults, figureDir);
writeReport(reportPath, tablePath, figureDir, trackingTable);

analysis = struct();
analysis.tracking = trackingTable;
analysis.tablePath = string(tablePath);
analysis.figureDir = string(figureDir);
analysis.reportPath = string(reportPath);

fprintf("QP recovery tracking table saved to:\n  %s\n", tablePath);
fprintf("QP recovery tracking report saved to:\n  %s\n", reportPath);
fprintf("Figures saved to:\n  %s\n", figureDir);
end

function caseResult = loadCase(resultDir, caseName)
filePath = fullfile(resultDir, caseName + ".mat");
loaded = load(filePath, "caseResult");
caseResult = loaded.caseResult;
end

function row = summarizeCase(caseResult, label, resultDir)
sig = caseResult.signals;
c = caseResult.case;
m = caseResult.metrics;

tauMax = [160, 160, 45];
uMax = [140, 140, 160];
hipMomentToTauSign = -1;

tBase = sig.time(:);
theta = sig.X(:, 3);
dtheta = sig.X(:, 6);
x = sig.X(:, 1);

tTau = sig.tau.time(:);
tau = sig.tau.data;
tU = sig.uLqr.time(:);
uLqr = sig.uLqr.data;
MBy = uLqr(:, 3);
tauHrefAtU = hipMomentToTauSign * MBy;
tauHAtU = interp1(tTau, tau(:, 1), tU, "linear", "extrap");
tauHError = tauHAtU - tauHrefAtU;

pulseEnd = c.pulseWindow(2);
recoveryEnd = min(max(tBase), pulseEnd + 0.5);
recoveryWindow = [pulseEnd, recoveryEnd];

inRecoveryU = tU >= recoveryWindow(1) & tU <= recoveryWindow(2);
inRecoveryTau = tTau >= recoveryWindow(1) & tTau <= recoveryWindow(2);
inRecoveryBase = tBase >= recoveryWindow(1) & tBase <= recoveryWindow(2);

tauRatio = abs(tau) ./ tauMax;
uRatio = abs(uLqr) ./ uMax;
qErrNorm = vecnorm(sig.legQError, 2, 2);
dqErrNorm = vecnorm(sig.legDqError, 2, 2);

thetaZeroTime = firstThetaZeroCrossing(tBase, theta, pulseEnd);
firstTheta15Time = firstAfter(tBase, abs(theta) > deg2rad(15), pulseEnd);
firstTauSatTime = firstAfter(tTau, max(tauRatio, [], 2) >= 0.95, pulseEnd);
firstULqrSatTime = firstAfter(tU, max(uRatio, [], 2) >= 0.95, pulseEnd);
firstTauHSatTime = firstAfter(tTau, tauRatio(:, 1) >= 0.95, pulseEnd);
firstMBySatTime = firstAfter(tU, uRatio(:, 3) >= 0.95, pulseEnd);

row = emptyRow();
row.label = string(label);
row.caseName = string(c.name);
row.resultDir = string(resultDir);
row.stable = logical(m.stable);
row.failureReason = string(m.failureReason);
row.recoveryStart = recoveryWindow(1);
row.recoveryEnd = recoveryWindow(2);

row.thetaPulseEndDeg = sampleAt(tBase, rad2deg(theta), pulseEnd);
row.dthetaPulseEnd = sampleAt(tBase, dtheta, pulseEnd);
row.thetaZeroTime = thetaZeroTime;
row.dthetaAtThetaZero = sampleAt(tBase, dtheta, thetaZeroTime);
row.xAtThetaZero = sampleAt(tBase, x, thetaZeroTime);
row.firstULqrSatTime = firstULqrSatTime;
row.firstMBySatTime = firstMBySatTime;
row.firstTauSatTime = firstTauSatTime;
row.firstTauHSatTime = firstTauHSatTime;
row.firstTheta15Time = firstTheta15Time;

row.recoveryMaxAbsMBy = maxOrNan(abs(MBy(inRecoveryU)));
row.recoveryMaxAbsTauHref = maxOrNan(abs(tauHrefAtU(inRecoveryU)));
row.recoveryMaxAbsTauH = maxOrNan(abs(tauHAtU(inRecoveryU)));
row.recoveryRmsTauHError = rmsOrNan(tauHError(inRecoveryU));
row.recoveryMaxAbsTauHError = maxOrNan(abs(tauHError(inRecoveryU)));
row.recoveryMeanAbsTauHError = meanOrNan(abs(tauHError(inRecoveryU)));
row.recoveryMBySatFraction = meanOrNan(uRatio(inRecoveryU, 3) >= 0.95);
row.recoveryTauHSatFraction = meanOrNan(tauRatio(inRecoveryTau, 1) >= 0.95);
row.recoveryAnyTauSatFraction = meanOrNan(max(tauRatio(inRecoveryTau, :), [], 2) >= 0.95);
row.recoveryAnyULqrSatFraction = meanOrNan(max(uRatio(inRecoveryU, :), [], 2) >= 0.95);
row.recoveryQErrorRms = rmsOrNan(qErrNorm(inRecoveryBase));
row.recoveryDqErrorRms = rmsOrNan(dqErrNorm(inRecoveryBase));
row.recoveryMaxQError = maxOrNan(qErrNorm(inRecoveryBase));
row.recoveryMaxDqError = maxOrNan(dqErrNorm(inRecoveryBase));

row.tauHErrorAtThetaZero = sampleAt(tU, tauHError, thetaZeroTime);
row.tauHrefAtThetaZero = sampleAt(tU, tauHrefAtU, thetaZeroTime);
row.tauHAtThetaZero = sampleAt(tU, tauHAtU, thetaZeroTime);
row.MByAtThetaZero = sampleAt(tU, MBy, thetaZeroTime);
row.tauRatioAtThetaZero = sampleAt(tTau, max(tauRatio, [], 2), thetaZeroTime);
row.uRatioAtThetaZero = sampleAt(tU, max(uRatio, [], 2), thetaZeroTime);

row.maxAbsThetaDeg = rad2deg(m.maxAbsTheta);
row.finalThetaDeg = rad2deg(m.finalTheta);
row.finalDtheta = m.finalDtheta;
row.finalX = m.finalX;
end

function row = emptyRow()
row = struct();
row.label = "";
row.caseName = "";
row.resultDir = "";
row.stable = false;
row.failureReason = "";
row.recoveryStart = NaN;
row.recoveryEnd = NaN;
row.thetaPulseEndDeg = NaN;
row.dthetaPulseEnd = NaN;
row.thetaZeroTime = NaN;
row.dthetaAtThetaZero = NaN;
row.xAtThetaZero = NaN;
row.firstULqrSatTime = NaN;
row.firstMBySatTime = NaN;
row.firstTauSatTime = NaN;
row.firstTauHSatTime = NaN;
row.firstTheta15Time = NaN;
row.recoveryMaxAbsMBy = NaN;
row.recoveryMaxAbsTauHref = NaN;
row.recoveryMaxAbsTauH = NaN;
row.recoveryRmsTauHError = NaN;
row.recoveryMaxAbsTauHError = NaN;
row.recoveryMeanAbsTauHError = NaN;
row.recoveryMBySatFraction = NaN;
row.recoveryTauHSatFraction = NaN;
row.recoveryAnyTauSatFraction = NaN;
row.recoveryAnyULqrSatFraction = NaN;
row.recoveryQErrorRms = NaN;
row.recoveryDqErrorRms = NaN;
row.recoveryMaxQError = NaN;
row.recoveryMaxDqError = NaN;
row.tauHErrorAtThetaZero = NaN;
row.tauHrefAtThetaZero = NaN;
row.tauHAtThetaZero = NaN;
row.MByAtThetaZero = NaN;
row.tauRatioAtThetaZero = NaN;
row.uRatioAtThetaZero = NaN;
row.maxAbsThetaDeg = NaN;
row.finalThetaDeg = NaN;
row.finalDtheta = NaN;
row.finalX = NaN;
end

function plotCases(caseResults, figureDir)
set(0, "DefaultFigureVisible", "off");
for idx = 1:numel(caseResults)
    cr = caseResults(idx).result;
    sig = cr.signals;
    c = cr.case;
    label = caseResults(idx).label;
    caseName = caseResults(idx).caseName;

    hipMomentToTauSign = -1;
    tTau = sig.tau.time(:);
    tau = sig.tau.data;
    tU = sig.uLqr.time(:);
    MBy = sig.uLqr.data(:, 3);
    tauHref = hipMomentToTauSign * MBy;
    tauHAtU = interp1(tTau, tau(:, 1), tU, "linear", "extrap");
    qErrNorm = vecnorm(sig.legQError, 2, 2);
    dqErrNorm = vecnorm(sig.legDqError, 2, 2);

    fig = figure("Name", label + "_" + caseName);
    tiledlayout(fig, 4, 1);

    ax = nexttile;
    plot(sig.time, rad2deg(sig.X(:, 3)), "LineWidth", 1.1);
    grid on;
    ylabel("theta (deg)");
    markPulseEnd(ax, c.pulseWindow(2));

    ax = nexttile;
    plot(tU, tauHref, "LineWidth", 1.1);
    hold on;
    plot(tU, tauHAtU, "LineWidth", 1.1);
    yline(160, ":");
    yline(-160, ":");
    grid on;
    ylabel("tau_h target/actual");
    legend("sign*MBy", "tau_h", "Location", "best");
    markPulseEnd(ax, c.pulseWindow(2));

    ax = nexttile;
    plot(tU, tauHAtU - tauHref, "LineWidth", 1.1);
    grid on;
    ylabel("tau_h error");
    markPulseEnd(ax, c.pulseWindow(2));

    ax = nexttile;
    plot(sig.time, qErrNorm, "LineWidth", 1.1);
    hold on;
    plot(sig.time, dqErrNorm, "LineWidth", 1.1);
    grid on;
    ylabel("leg error norm");
    xlabel("time (s)");
    legend("q", "dq", "Location", "best");
    markPulseEnd(ax, c.pulseWindow(2));

    fileName = safeFileStem(label + "_" + caseName) + ".png";
    saveas(fig, fullfile(figureDir, fileName));
    close(fig);
end
end

function writeReport(reportPath, tablePath, figureDir, trackingTable)
fid = fopen(reportPath, "w");
if fid < 0
    error("analyze_qp_recovery_tracking:CannotWriteReport", ...
        "Could not write report: %s", reportPath);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# QP Recovery Tracking Analysis\n\n");
fprintf(fid, "This report checks whether lower-layer QP hip torque tracks the upper-layer pitch moment command during the post-pulse recovery window.\n\n");
fprintf(fid, "Tracking table:\n\n```text\n%s\n```\n\n", tablePath);
fprintf(fid, "Figures:\n\n```text\n%s\n```\n\n", figureDir);

fprintf(fid, "## Key Metrics\n\n");
fprintf(fid, "| label | case | MBy sat frac | tau_h sat frac | tau_h RMS err | tau_h max err | q RMS | dq RMS | theta zero t | tau err @ zero | max theta deg | final theta deg |\n");
fprintf(fid, "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n");
for idx = 1:height(trackingTable)
    fprintf(fid, "| %s | %s | %.2f | %.2f | %.2f | %.2f | %.3g | %.3g | %.3g | %.2f | %.1f | %.1f |\n", ...
        trackingTable.label(idx), trackingTable.caseName(idx), ...
        trackingTable.recoveryMBySatFraction(idx), ...
        trackingTable.recoveryTauHSatFraction(idx), ...
        trackingTable.recoveryRmsTauHError(idx), ...
        trackingTable.recoveryMaxAbsTauHError(idx), ...
        trackingTable.recoveryQErrorRms(idx), ...
        trackingTable.recoveryDqErrorRms(idx), ...
        trackingTable.thetaZeroTime(idx), ...
        trackingTable.tauHErrorAtThetaZero(idx), ...
        trackingTable.maxAbsThetaDeg(idx), ...
        trackingTable.finalThetaDeg(idx));
end

fprintf(fid, "\n## Interpretation Guide\n\n");
fprintf(fid, "If `MBy sat frac` is high before `tau_h sat frac`, the upper layer is asking for saturated pitch recovery before the lower layer hits its own hip limit. ");
fprintf(fid, "If `tau_h RMS err` is large while `MBy sat frac` is low, the QP is sacrificing moment tracking to satisfy dynamics, contact, or joint tracking. ");
fprintf(fid, "If both are high, the recovery request is outside the combined upper/lower authority envelope.\n\n");

fprintf(fid, "Use this report to decide whether the next change should be LQR wrench shaping or QP priority tuning.\n");
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

function value = sampleAt(t, y, queryTime)
if ~isfinite(queryTime)
    value = NaN;
else
    value = interp1(t, y, queryTime, "linear", "extrap");
end
end

function value = maxOrNan(x)
if isempty(x)
    value = NaN;
else
    value = max(x, [], "all");
end
end

function value = meanOrNan(x)
if isempty(x)
    value = NaN;
else
    value = mean(double(x), "all");
end
end

function value = rmsOrNan(x)
if isempty(x)
    value = NaN;
else
    value = rms(x, "all");
end
end

function markPulseEnd(ax, pulseEnd)
if isfinite(pulseEnd)
    axes(ax); %#ok<LAXES>
    xline(pulseEnd, ":");
end
end

function ensureFolder(path)
if ~isfolder(path)
    mkdir(path);
end
end

function stem = safeFileStem(name)
stem = regexprep(char(string(name)), "[^a-zA-Z0-9_\-]", "_");
end
