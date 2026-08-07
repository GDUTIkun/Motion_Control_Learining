function analysis = analyze_leg_branch_geometry(resultDirs, labels, caseNames)
%ANALYZE_LEG_BRANCH_GEOMETRY Inspect leg branch/singularity behavior.
%
% This script does not run Simulink. It loads existing caseResult MAT files
% and quantifies whether the actual leg approaches a two-link singularity or
% moves to the opposite side of the hip during recovery.

if nargin < 1 || isempty(resultDirs)
    root = fullfile("D:\Workspace\CodeWorkspace\calibration", ...
        "results", "studies", "2026_08_lqr_disturbance_response");
    resultDirs = [
        fullfile(root, "20260807_182335")
        fullfile(root, "20260807_203047")
        ];
end
if nargin < 2 || isempty(labels)
    labels = ["round2_best"; "force0p8"];
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
repoRoot = char(java.io.File(fullfile(calibrationDir, "..")).getCanonicalPath());
simDir = fullfile(repoRoot, "dynamic", "2D", "simulate", "single_wheel_leg");
addpath(simDir);

processedDir = fullfile(calibrationDir, "data", "processed");
reportPath = fullfile(calibrationDir, "reports", ...
    "2026_08_leg_branch_geometry.md");
tablePath = fullfile(processedDir, "2026_08_leg_branch_geometry.csv");
ensureFolder(processedDir);

originalDir = pwd;
cleanup = onCleanup(@() cd(originalDir));
cd(simDir);
run("startup.m");
legLocal = evalin("base", "leg");

rows = repmat(emptyRow(), 0, 1);
for resultIdx = 1:numel(resultDirs)
    resultDir = char(resultDirs(resultIdx));
    for caseIdx = 1:numel(caseNames)
        caseName = string(caseNames(caseIdx));
        caseResult = loadCase(resultDir, caseName);
        rows(end + 1, 1) = summarizeCase(caseResult, labels(resultIdx), ...
            resultDir, legLocal); %#ok<AGROW>
    end
end

geometryTable = struct2table(rows);
writetable(geometryTable, tablePath);
writeReport(reportPath, tablePath, geometryTable);

analysis = struct();
analysis.geometry = geometryTable;
analysis.tablePath = string(tablePath);
analysis.reportPath = string(reportPath);

fprintf("Leg branch geometry table saved to:\n  %s\n", tablePath);
fprintf("Leg branch geometry report saved to:\n  %s\n", reportPath);
disp(geometryTable);
end

function caseResult = loadCase(resultDir, caseName)
filePath = fullfile(resultDir, caseName + ".mat");
loaded = load(filePath, "caseResult");
caseResult = loaded.caseResult;
end

function row = summarizeCase(caseResult, label, resultDir, legLocal)
sig = caseResult.signals;
c = caseResult.case;
m = caseResult.metrics;

t = sig.time(:);
theta = sig.X(:, 3);
dtheta = sig.X(:, 6);
qRel = sig.qRel;
dqRel = sig.dqRel;
qRef = sig.qRef;
dqRef = sig.dqRef;

qAbs = [qRel(:, 1) + theta, qRel(:, 2), qRel(:, 3)];
dqAbs = [dqRel(:, 1) + dtheta, dqRel(:, 2), dqRel(:, 3)];

n = numel(t);
pO = zeros(n, 2);
detJO = zeros(n, 1);
for idx = 1:n
    kin = wheel_leg_kinematics(qAbs(idx, :).', dqAbs(idx, :).', ...
        zeros(3, 1), legLocal);
    pO(idx, :) = kin.pO(:).';
    detJO(idx) = det(kin.JO(:, 1:2));
end

pulseEnd = c.pulseWindow(2);
post = t >= pulseEnd;

[minAbsSinQkPost, minIdxRel] = min(abs(sin(qAbs(post, 2))));
postIdx = find(post);
minIdx = postIdx(minIdxRel);

qErrNorm = vecnorm(qRel(:, 1:2) - qRef(:, 1:2), 2, 2);
dqErrNorm = vecnorm(dqRel(:, 1:2) - dqRef(:, 1:2), 2, 2);

row = emptyRow();
row.label = string(label);
row.caseName = string(c.name);
row.resultDir = string(resultDir);
row.stable = logical(m.stable);
row.maxAbsThetaDeg = rad2deg(m.maxAbsTheta);
row.finalThetaDeg = rad2deg(m.finalTheta);
row.minAbsSinQkPost = minAbsSinQkPost;
row.minAbsDetJPost = min(abs(detJO(post)));
row.qkAtMinDetDeg = rad2deg(qAbs(minIdx, 2));
row.timeAtMinDet = t(minIdx);
row.pOxAtMinDet = pO(minIdx, 1);
row.maxAbsQkDegPost = max(abs(rad2deg(qAbs(post, 2))));
row.minQkDegPost = min(rad2deg(qAbs(post, 2)));
row.maxQkDegPost = max(rad2deg(qAbs(post, 2)));
row.pOxMinPost = min(pO(post, 1));
row.pOxMaxPost = max(pO(post, 1));
row.pOxFinal = pO(end, 1);
row.qErrMaxPost = max(qErrNorm(post));
row.dqErrMaxPost = max(dqErrNorm(post));
row.qErrRmsPost = rms(qErrNorm(post));
row.dqErrRmsPost = rms(dqErrNorm(post));
end

function row = emptyRow()
row = struct();
row.label = "";
row.caseName = "";
row.resultDir = "";
row.stable = false;
row.maxAbsThetaDeg = NaN;
row.finalThetaDeg = NaN;
row.minAbsSinQkPost = NaN;
row.minAbsDetJPost = NaN;
row.qkAtMinDetDeg = NaN;
row.timeAtMinDet = NaN;
row.pOxAtMinDet = NaN;
row.maxAbsQkDegPost = NaN;
row.minQkDegPost = NaN;
row.maxQkDegPost = NaN;
row.pOxMinPost = NaN;
row.pOxMaxPost = NaN;
row.pOxFinal = NaN;
row.qErrMaxPost = NaN;
row.dqErrMaxPost = NaN;
row.qErrRmsPost = NaN;
row.dqErrRmsPost = NaN;
end

function writeReport(reportPath, tablePath, geometryTable)
fid = fopen(reportPath, "w");
if fid < 0
    warning("analyze_leg_branch_geometry:ReadmeOpenFailed", ...
        "Could not write report: %s.", reportPath);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# 2026-08 Leg Branch Geometry Diagnostic\n\n");
fprintf(fid, "Data table:\n\n```text\n%s\n```\n\n", tablePath);
fprintf(fid, "This analysis checks actual leg geometry after the disturbance pulse. ");
fprintf(fid, "For the two-link leg, the wheel-center Jacobian determinant is ");
fprintf(fid, "`L1*L2*sin(qk)`, so `qk` near `0 deg` or `180 deg` is singular.\n\n");

fprintf(fid, "Summary:\n\n```text\n");
dispText = evalc("disp(geometryTable)");
fprintf(fid, "%s", dispText);
fprintf(fid, "```\n\n");

fprintf(fid, "Use this report to decide whether the next fix should add a leg ");
fprintf(fid, "configuration guard, a QP posture priority, or an IK branch constraint.\n");
end

function ensureFolder(pathName)
if ~isfolder(pathName)
    mkdir(pathName);
end
end
