function analysis = analyze_pass_fail_cases(resultDir)
%ANALYZE_PASS_FAIL_CASES Compare representative passing/failing cases.
%
% This script does not run Simulink. It loads existing study results and
% compares pitch_10deg_pulse_5N against pitch_10deg_pulse_10N.

if nargin < 1 || isempty(resultDir)
    resultDir = fullfile("D:\Workspace\CodeWorkspace\calibration", ...
        "results", "studies", "2026_08_lqr_disturbance_response", ...
        "20260806_195701");
end

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());
figureDir = fullfile(calibrationDir, "figures", "studies", ...
    "2026_08_lqr_disturbance_response", "pass_fail_20260806_195701");
reportPath = fullfile(calibrationDir, "reports", ...
    "2026_08_pass_fail_mechanism.md");

if ~isfolder(figureDir)
    mkdir(figureDir);
end

passCase = loadCase(resultDir, "pitch_10deg_pulse_5N");
failCase = loadCase(resultDir, "pitch_10deg_pulse_10N");

analysis = struct();
analysis.resultDir = string(resultDir);
analysis.figureDir = string(figureDir);
analysis.reportPath = string(reportPath);
analysis.pass = summarizeCase(passCase);
analysis.fail = summarizeCase(failCase);
analysis.comparison = compareCases(analysis.pass, analysis.fail);

plotComparison(passCase, failCase, figureDir);
writeReport(analysis, reportPath);

fprintf("Pass/fail mechanism report saved to:\n  %s\n", reportPath);
fprintf("Figures saved to:\n  %s\n", figureDir);
end

function caseResult = loadCase(resultDir, caseName)
filePath = fullfile(resultDir, caseName + ".mat");
loaded = load(filePath, "caseResult");
caseResult = loaded.caseResult;
end

function s = summarizeCase(caseResult)
sig = caseResult.signals;
m = caseResult.metrics;
c = caseResult.case;

theta = sig.X(:, 3);
dtheta = sig.X(:, 6);
x = sig.X(:, 1);
tau = sig.tau.data;
uLqr = sig.uLqr.data;

tauMax = [160, 160, 45];
tauRatio = abs(tau) ./ tauMax;
tauSatAny = max(tauRatio, [], 2) >= 0.95;

s = struct();
s.name = string(c.name);
s.stable = logical(m.stable);
s.failureReason = string(m.failureReason);
s.maxAbsThetaDeg = rad2deg(m.maxAbsTheta);
s.finalThetaDeg = rad2deg(m.finalTheta);
s.finalDtheta = m.finalDtheta;
s.finalX = m.finalX;
s.maxAbsX = m.maxAbsX;
s.settlingTimeTheta = m.settlingTimeTheta;
s.tauSaturationRatio = m.tauSaturationRatio;
s.pulseMaxAbsThetaDeg = rad2deg(m.pulseMaxAbsTheta);
s.pulseMaxAbsTau = m.pulseMaxAbsTau;
s.pulseMaxAbsULqr = m.pulseMaxAbsULqr;
s.maxAbsFHx = max(abs(uLqr(:, 1)));
s.maxAbsFHz = max(abs(uLqr(:, 2)));
s.maxAbsMBy = max(abs(uLqr(:, 3)));
s.finalThetaRateDeg = rad2deg(dtheta(end));
s.finalXRate = sig.X(end, 4);
s.firstTauSatTime = firstTrueTime(sig.tau.time, tauSatAny);
s.firstTheta15DegTime = firstTrueTime(sig.time, abs(theta) > deg2rad(15));
s.firstAbsX05Time = firstTrueTime(sig.time, abs(x) > 0.5);
s.thetaAtPulseStartDeg = interp1(sig.time, rad2deg(theta), c.pulseWindow(1), ...
    "linear", "extrap");
s.thetaAtPulseEndDeg = interp1(sig.time, rad2deg(theta), c.pulseWindow(2), ...
    "linear", "extrap");
s.xAtPulseEnd = interp1(sig.time, x, c.pulseWindow(2), "linear", "extrap");
end

function t = firstTrueTime(time, mask)
idx = find(mask, 1, "first");
if isempty(idx)
    t = NaN;
else
    t = time(idx);
end
end

function comparison = compareCases(passCase, failCase)
comparison = struct();
comparison.deltaMaxAbsThetaDeg = failCase.maxAbsThetaDeg - passCase.maxAbsThetaDeg;
comparison.deltaFinalX = failCase.finalX - passCase.finalX;
comparison.deltaTauSaturationRatio = failCase.tauSaturationRatio ...
    - passCase.tauSaturationRatio;
comparison.failFirstLimit = classifyFirstLimit(failCase);
end

function label = classifyFirstLimit(s)
events = [
    struct("name", "theta > 15 deg", "time", s.firstTheta15DegTime)
    struct("name", "|x| > 0.5 m", "time", s.firstAbsX05Time)
    struct("name", "tau saturation", "time", s.firstTauSatTime)
];
times = [events.time];
valid = isfinite(times);
if ~any(valid)
    label = "none";
    return;
end

validEvents = events(valid);
[~, idx] = min([validEvents.time]);
label = string(validEvents(idx).name);
end

function plotComparison(passCase, failCase, figureDir)
set(0, "DefaultFigureVisible", "off");
plotBaseComparison(passCase, failCase, figureDir);
plotCommandComparison(passCase, failCase, figureDir);
plotTauComparison(passCase, failCase, figureDir);
plotLegErrorComparison(passCase, failCase, figureDir);
end

function plotBaseComparison(passCase, failCase, figureDir)
fig = figure("Name", "pass_fail_base");
tiledlayout(fig, 3, 1);
plotPair(nexttile, passCase, failCase, @(s) rad2deg(s.X(:, 3)), "thetaB (deg)");
plotPair(nexttile, passCase, failCase, @(s) rad2deg(s.X(:, 6)), "dthetaB (deg/s)");
plotPair(nexttile, passCase, failCase, @(s) s.X(:, 1), "xB (m)");
xlabel("time (s)");
saveas(fig, fullfile(figureDir, "pass_fail_base.png"));
close(fig);
end

function plotCommandComparison(passCase, failCase, figureDir)
fig = figure("Name", "pass_fail_lqr");
tiledlayout(fig, 3, 1);
labels = ["FHx_ext (N)", "FHz_ext (N)", "MBy_des (N*m)"];
for i = 1:3
    ax = nexttile;
    plot(passCase.signals.uLqr.time, passCase.signals.uLqr.data(:, i), ...
        "LineWidth", 1.1);
    hold on;
    plot(failCase.signals.uLqr.time, failCase.signals.uLqr.data(:, i), ...
        "LineWidth", 1.1);
    grid on;
    ylabel(labels(i));
    legend("5N pass", "10N fail", "Location", "best");
end
xlabel("time (s)");
saveas(fig, fullfile(figureDir, "pass_fail_lqr.png"));
close(fig);
end

function plotTauComparison(passCase, failCase, figureDir)
fig = figure("Name", "pass_fail_tau");
tiledlayout(fig, 3, 1);
labels = ["tau_h", "tau_k", "tau_w"];
limits = [160, 160, 45];
for i = 1:3
    ax = nexttile;
    plot(passCase.signals.tau.time, passCase.signals.tau.data(:, i), ...
        "LineWidth", 1.1);
    hold on;
    plot(failCase.signals.tau.time, failCase.signals.tau.data(:, i), ...
        "LineWidth", 1.1);
    grid on;
    ylabel(labels(i));
    legend("5N pass", "10N fail", "Location", "best");
    yline(limits(i), ":");
    yline(-limits(i), ":");
end
xlabel("time (s)");
saveas(fig, fullfile(figureDir, "pass_fail_tau.png"));
close(fig);
end

function plotLegErrorComparison(passCase, failCase, figureDir)
fig = figure("Name", "pass_fail_leg_error");
tiledlayout(fig, 2, 1);
plotPair(nexttile, passCase, failCase, @(s) vecnorm(s.legQError, 2, 2), ...
    "hip/knee q error norm (rad)");
plotPair(nexttile, passCase, failCase, @(s) vecnorm(s.legDqError, 2, 2), ...
    "hip/knee dq error norm (rad/s)");
xlabel("time (s)");
saveas(fig, fullfile(figureDir, "pass_fail_leg_error.png"));
close(fig);
end

function plotPair(ax, passCase, failCase, selector, yLabelText)
axes(ax);
plot(passCase.signals.time, selector(passCase.signals), "LineWidth", 1.1);
hold on;
plot(failCase.signals.time, selector(failCase.signals), "LineWidth", 1.1);
grid on;
ylabel(yLabelText);
legend("5N pass", "10N fail", "Location", "best");
end

function writeReport(analysis, reportPath)
fid = fopen(reportPath, "w");
if fid < 0
    error("analyze_pass_fail_cases:CannotWriteReport", ...
        "Could not write report: %s", reportPath);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Pass/Fail Mechanism Analysis\n\n");
fprintf(fid, "Result directory:\n\n```text\n%s\n```\n\n", analysis.resultDir);

writeCase(fid, "Passing representative", analysis.pass);
writeCase(fid, "Failing representative", analysis.fail);

fprintf(fid, "## Comparison\n\n");
fprintf(fid, "- Delta max |theta|: %.3g deg\n", ...
    analysis.comparison.deltaMaxAbsThetaDeg);
fprintf(fid, "- Delta final x: %.3g m\n", analysis.comparison.deltaFinalX);
fprintf(fid, "- Delta tau saturation ratio: %.3g\n", ...
    analysis.comparison.deltaTauSaturationRatio);
fprintf(fid, "- First detected failing limit: %s\n\n", ...
    analysis.comparison.failFirstLimit);

fprintf(fid, "## Interpretation\n\n");
fprintf(fid, "The 10 N pulse case crosses the accepted pitch/x envelope and later reaches torque saturation. ");
fprintf(fid, "The 5 N pulse case remains inside the envelope with large torque margin. ");
fprintf(fid, "This suggests the first tuning pass should compare LQR horizontal-force use, x-drift penalty, and pitch damping before changing architecture.\n\n");

fprintf(fid, "## Tuning Hypotheses\n\n");
fprintf(fid, "1. If the failing case shows excessive x drift before torque saturation, increase x/dx penalty or increase horizontal-force cost.\n");
fprintf(fid, "2. If pitch rate remains high after the pulse while torque has margin, increase theta/dtheta penalty or reduce moment cost.\n");
fprintf(fid, "3. If torque saturation occurs early, reduce LQR aggressiveness or adjust QP torque/wrench priorities.\n");
fprintf(fid, "4. Keep the fixed 9-case acceptance set as the regression baseline.\n");
end

function writeCase(fid, titleText, s)
fprintf(fid, "## %s\n\n", titleText);
fprintf(fid, "- Case: `%s`\n", s.name);
fprintf(fid, "- Stable: %d\n", s.stable);
fprintf(fid, "- Failure reason: %s\n", s.failureReason);
fprintf(fid, "- max |theta|: %.3g deg\n", s.maxAbsThetaDeg);
fprintf(fid, "- final theta: %.3g deg\n", s.finalThetaDeg);
fprintf(fid, "- final dtheta: %.3g rad/s\n", s.finalDtheta);
fprintf(fid, "- final x: %.3g m\n", s.finalX);
fprintf(fid, "- tau saturation ratio: %.3g\n", s.tauSaturationRatio);
fprintf(fid, "- max |FHx|: %.3g N\n", s.maxAbsFHx);
fprintf(fid, "- max |FHz|: %.3g N\n", s.maxAbsFHz);
fprintf(fid, "- max |MBy|: %.3g N*m\n", s.maxAbsMBy);
fprintf(fid, "- first theta > 15 deg: %.3g s\n", s.firstTheta15DegTime);
fprintf(fid, "- first |x| > 0.5 m: %.3g s\n", s.firstAbsX05Time);
fprintf(fid, "- first tau saturation: %.3g s\n\n", s.firstTauSatTime);
end
