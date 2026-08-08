function analysis = analyze_boundary_guard_batch(batchDir)
%ANALYZE_BOUNDARY_GUARD_BATCH Summarize a boundary guard grid batch.
%
% This script does not run Simulink. It reads the batch created by
% validate_boundary_guard_grid and writes a compact decision table.
%
% Usage:
%   analysis = analyze_boundary_guard_batch;
%   analysis = analyze_boundary_guard_batch("D:\...\boundary_guard_grid_YYYYMMDD_HHMMSS");

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());
root = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_lqr_disturbance_response");

if nargin < 1 || strlength(string(batchDir)) == 0
    batchDir = latestBatchDir(root);
end
batchDir = char(java.io.File(batchDir).getCanonicalPath());

batchFile = fullfile(batchDir, "boundary_guard_grid_batch.mat");
if ~isfile(batchFile)
    error("analyze_boundary_guard_batch:MissingBatch", ...
        "Could not find boundary_guard_grid_batch.mat in: %s", batchDir);
end

loaded = load(batchFile, "batch");
batch = loaded.batch;

processedDir = fullfile(calibrationDir, "data", "processed");
reportPath = fullfile(calibrationDir, "reports", ...
    "2026_08_boundary_guard_grid_summary.md");
tablePath = fullfile(processedDir, "2026_08_boundary_guard_grid_summary.csv");
ensureFolder(processedDir);

decisionTable = makeDecisionTable(batch.results);
writetable(decisionTable, tablePath);
writeReport(reportPath, batchDir, tablePath, decisionTable);

analysis = struct();
analysis.batchDir = string(batchDir);
analysis.decisionTable = decisionTable;
analysis.tablePath = string(tablePath);
analysis.reportPath = string(reportPath);

fprintf("Boundary guard decision table saved to:\n  %s\n", tablePath);
fprintf("Boundary guard report saved to:\n  %s\n", reportPath);
disp(decisionTable);
end

function batchDir = latestBatchDir(root)
items = dir(fullfile(root, "boundary_guard_grid_*"));
items = items([items.isdir]);
if isempty(items)
    error("analyze_boundary_guard_batch:NoBatch", ...
        "No boundary_guard_grid_* directory found in: %s", root);
end
[~, order] = sort([items.datenum], "descend");
batchDir = fullfile(items(order(1)).folder, items(order(1)).name);
end

function decisionTable = makeDecisionTable(results)
n = numel(results);
variant = strings(n, 1);
resultDir = strings(n, 1);
goodCount = zeros(n, 1);
stableCount = zeros(n, 1);
branchOkCount = zeros(n, 1);
good8N = zeros(n, 1);
good8p5N = zeros(n, 1);
good9N = zeros(n, 1);
maxTheta8p5N = zeros(n, 1);
maxTheta9N = zeros(n, 1);
maxAbsPOx = zeros(n, 1);
minAbsSinQk = zeros(n, 1);
maxLegDqErr = zeros(n, 1);
maxTauSat = zeros(n, 1);

for idx = 1:n
    s = results(idx).summary;
    variant(idx) = results(idx).variant.name;
    resultDir(idx) = results(idx).resultDir;

    stable = logical(s.stable);
    branchOk = logical(s.legBranchOk);
    good = stable & branchOk;

    is8N = abs(s.pulseAmplitudeN - 8) < 1e-9;
    is8p5N = abs(s.pulseAmplitudeN - 8.5) < 1e-9;
    is9N = abs(s.pulseAmplitudeN - 9) < 1e-9;

    goodCount(idx) = sum(good);
    stableCount(idx) = sum(stable);
    branchOkCount(idx) = sum(branchOk);
    good8N(idx) = sum(good & is8N);
    good8p5N(idx) = sum(good & is8p5N);
    good9N(idx) = sum(good & is9N);
    maxTheta8p5N(idx) = maxOrNan(s.maxAbsThetaDeg(is8p5N));
    maxTheta9N(idx) = maxOrNan(s.maxAbsThetaDeg(is9N));
    maxAbsPOx(idx) = max(s.maxAbsPOxPost);
    minAbsSinQk(idx) = min(s.minAbsSinQkPost);
    maxLegDqErr(idx) = max(s.maxLegDqErrorPost);
    maxTauSat(idx) = max(s.tauSaturationRatio);
end

decisionTable = table(variant, goodCount, stableCount, branchOkCount, ...
    good8N, good8p5N, good9N, maxTheta8p5N, maxTheta9N, ...
    maxAbsPOx, minAbsSinQk, maxLegDqErr, maxTauSat, resultDir);
decisionTable = sortrows(decisionTable, ...
    ["goodCount", "good9N", "good8p5N", "maxTheta9N"], ...
    ["descend", "descend", "descend", "ascend"]);
end

function value = maxOrNan(x)
if isempty(x)
    value = NaN;
else
    value = max(x);
end
end

function writeReport(reportPath, batchDir, tablePath, decisionTable)
fid = fopen(reportPath, "w");
if fid < 0
    warning("analyze_boundary_guard_batch:ReadmeOpenFailed", ...
        "Could not write report: %s.", reportPath);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# 2026-08 Boundary Guard Grid Summary\n\n");
fprintf(fid, "Batch:\n\n```text\n%s\n```\n\n", batchDir);
fprintf(fid, "Decision table:\n\n```text\n%s\n```\n\n", tablePath);
fprintf(fid, "A case is counted as good only when both the original stability ");
fprintf(fid, "criteria pass and `legBranchOk` remains true.\n\n");
fprintf(fid, "Summary:\n\n```text\n");
dispText = evalc("disp(decisionTable)");
fprintf(fid, "%s", dispText);
fprintf(fid, "```\n");
end

function ensureFolder(pathName)
if ~isfolder(pathName)
    mkdir(pathName);
end
end
