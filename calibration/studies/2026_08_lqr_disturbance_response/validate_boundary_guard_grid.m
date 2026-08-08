function batch = validate_boundary_guard_grid(bestResultDir, variants)
%VALIDATE_BOUNDARY_GUARD_GRID Run boundary cases with leg-guard candidates.
%
% The default batch focuses on the observed boundary:
%   theta0 = [-10, 0, 10] deg
%   pulse  = [8, 8.5, 9] N
%   stop   = 8 s
%
% It keeps the round-2 best upper LQR Q/R and tests a small set of lower-layer
% and limit variants intended to prevent leg branch flips near recovery.
%
% Usage:
%   batch = validate_boundary_guard_grid;
%   batch = validate_boundary_guard_grid("D:\...\single_wheel_leg_lqr_qr_round2\20260806_224811");

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());

if nargin < 1 || strlength(string(bestResultDir)) == 0
    bestResultDir = fullfile(calibrationDir, "results", ...
        "single_wheel_leg_lqr_qr_round2", "20260806_224811");
end
if nargin < 2 || isempty(variants)
    variants = defaultVariants();
end

bestResultDir = char(java.io.File(bestResultDir).getCanonicalPath());
bestParamFile = fullfile(bestResultDir, "best_params.mat");
if ~isfile(bestParamFile)
    error("validate_boundary_guard_grid:MissingBestParams", ...
        "Could not find best_params.mat in: %s", bestResultDir);
end

loaded = load(bestParamFile, "bestParams", "bestObjective");
if ~isfield(loaded, "bestParams")
    error("validate_boundary_guard_grid:InvalidBestParams", ...
        "best_params.mat does not contain bestParams.");
end

addpath(studyDir);
cases = boundary_pulse_cases();

batchStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
batchDir = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_lqr_disturbance_response", "boundary_guard_grid_" + batchStamp);
if ~isfolder(batchDir)
    mkdir(batchDir);
end

batch = struct();
batch.batchDir = string(batchDir);
batch.bestResultDir = string(bestResultDir);
batch.bestObjective = loaded.bestObjective;
batch.cases = cases;
batch.variants = variants;
batch.results = repmat(struct("variant", [], "resultDir", "", "summary", []), ...
    numel(variants), 1);

for idx = 1:numel(variants)
    variant = variants(idx);
    fprintf("\nRunning boundary guard variant %d/%d: %s\n", ...
        idx, numel(variants), variant.name);
    printVariant(variant);

    out = run_cases(cases, loaded.bestParams, ...
        variant.limitParams, variant.ctrlParams);
    metadata = makeMetadata(bestResultDir, loaded, variant, batchDir, cases);
    save(fullfile(out.resultDir, "validation_metadata.mat"), "metadata");
    writeVariantReadme(out.resultDir, metadata);

    batch.results(idx).variant = variant;
    batch.results(idx).resultDir = string(out.resultDir);
    batch.results(idx).summary = out.summary;
end

indexTable = makeIndexTable(batch.results);
writetable(indexTable, fullfile(batchDir, "boundary_guard_grid_index.csv"));
save(fullfile(batchDir, "boundary_guard_grid_batch.mat"), ...
    "batch", "indexTable");
writeBatchReadme(batchDir, batch, indexTable);

fprintf("\nBoundary guard grid batch saved to:\n%s\n", batchDir);
disp(indexTable);
end

function variants = defaultVariants()
variants = repmat(emptyVariant(), 6, 1);

variants(1) = variantDef("round2_baseline", [], [], ...
    "Round-2 best Q/R, current limits and lower QP settings.");

variants(2) = variantDef("force_0p8", ...
    limits(1.0, 0.8, 1.0), [], ...
    "Reduce only Fx/Fz limits to 80 percent.");

variants(3) = variantDef("wqdd_leg_3x", [], ...
    ctrl("qpWqddScale", [3; 3; 1]), ...
    "Increase hip/knee acceleration tracking priority in the QP.");

variants(4) = variantDef("force_0p8_wqdd_leg_3x", ...
    limits(1.0, 0.8, 1.0), ctrl("qpWqddScale", [3; 3; 1]), ...
    "Combine milder Fx/Fz authority with stronger hip/knee QP tracking.");

variants(5) = variantDef("force_0p8_pd_1p5_wqdd_leg_3x", ...
    limits(1.0, 0.8, 1.0), ...
    ctrl("KpScale", [1.5; 1.5; 1], ...
         "KdScale", [1.5; 1.5; 1], ...
         "qpWqddScale", [3; 3; 1]), ...
    "Add stronger hip/knee PD tracking on top of the QP tracking priority.");

variants(6) = variantDef("force_0p8_tau_1p2_wqdd_leg_3x", ...
    limits(1.2, 0.8, 1.0), ctrl("qpWqddScale", [3; 3; 1]), ...
    "Check whether mild extra joint torque authority prevents branch loss.");
end

function v = emptyVariant()
v = struct();
v.name = "";
v.limitParams = [];
v.ctrlParams = [];
v.notes = "";
end

function v = variantDef(name, limitParams, ctrlParams, notes)
v = emptyVariant();
v.name = string(name);
v.limitParams = limitParams;
v.ctrlParams = ctrlParams;
v.notes = string(notes);
end

function s = limits(tauScale, forceScale, momentScale)
s = struct( ...
    "tauScale", double(tauScale), ...
    "forceScale", double(forceScale), ...
    "momentScale", double(momentScale));
end

function s = ctrl(varargin)
s = struct();
for idx = 1:2:numel(varargin)
    s.(char(varargin{idx})) = varargin{idx + 1};
end
end

function printVariant(variant)
disp("  notes: " + variant.notes);
if ~isempty(variant.limitParams)
    fprintf("  limit tau/force/moment = %.4g / %.4g / %.4g\n", ...
        variant.limitParams.tauScale, variant.limitParams.forceScale, ...
        variant.limitParams.momentScale);
end
if ~isempty(variant.ctrlParams)
    disp("  ctrl fields:");
    disp(fieldnames(variant.ctrlParams));
end
end

function metadata = makeMetadata(bestResultDir, loaded, variant, batchDir, cases)
metadata = struct();
metadata.validationType = "boundary_guard_grid";
metadata.bestResultDir = string(bestResultDir);
metadata.bestObjective = loaded.bestObjective;
metadata.variant = variant;
metadata.limitParams = variant.limitParams;
metadata.ctrlParams = variant.ctrlParams;
metadata.batchDir = string(batchDir);
metadata.cases = cases;
metadata.createdAt = datetime("now");
end

function indexTable = makeIndexTable(results)
n = numel(results);
variant = strings(n, 1);
resultDir = strings(n, 1);
stableCount = zeros(n, 1);
branchOkCount = zeros(n, 1);
stableAndBranchOkCount = zeros(n, 1);
failedOrBranchBadCount = zeros(n, 1);
maxFinalAbsTheta = zeros(n, 1);
maxAbsTheta = zeros(n, 1);
maxAbsPOxPost = zeros(n, 1);
minAbsSinQkPost = zeros(n, 1);
maxLegQErrorPost = zeros(n, 1);
maxLegDqErrorPost = zeros(n, 1);
maxTauSaturation = zeros(n, 1);

for idx = 1:n
    s = results(idx).summary;
    variant(idx) = results(idx).variant.name;
    resultDir(idx) = results(idx).resultDir;
    stable = logical(s.stable);
    branchOk = logical(s.legBranchOk);
    stableCount(idx) = sum(stable);
    branchOkCount(idx) = sum(branchOk);
    stableAndBranchOkCount(idx) = sum(stable & branchOk);
    failedOrBranchBadCount(idx) = sum(~(stable & branchOk));
    maxFinalAbsTheta(idx) = max(abs(s.finalThetaDeg));
    maxAbsTheta(idx) = max(s.maxAbsThetaDeg);
    maxAbsPOxPost(idx) = max(s.maxAbsPOxPost);
    minAbsSinQkPost(idx) = min(s.minAbsSinQkPost);
    maxLegQErrorPost(idx) = max(s.maxLegQErrorPost);
    maxLegDqErrorPost(idx) = max(s.maxLegDqErrorPost);
    maxTauSaturation(idx) = max(s.tauSaturationRatio);
end

indexTable = table(variant, stableCount, branchOkCount, ...
    stableAndBranchOkCount, failedOrBranchBadCount, maxFinalAbsTheta, ...
    maxAbsTheta, maxAbsPOxPost, minAbsSinQkPost, maxLegQErrorPost, ...
    maxLegDqErrorPost, maxTauSaturation, resultDir);
end

function writeBatchReadme(batchDir, batch, indexTable)
fid = fopen(fullfile(batchDir, "README.md"), "w");
if fid < 0
    warning("validate_boundary_guard_grid:ReadmeOpenFailed", ...
        "Could not write README.md in %s.", batchDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Boundary Guard Grid\n\n");
fprintf(fid, "Source best QR:\n\n```text\n%s\n```\n\n", batch.bestResultDir);
fprintf(fid, "Cases:\n\n```text\ntheta0 = [-10, 0, 10] deg\npulse  = [8, 8.5, 9] N\nstop   = 8 s\n```\n\n");
fprintf(fid, "Index table:\n\n```text\nboundary_guard_grid_index.csv\n```\n\n");
fprintf(fid, "## Variants\n\n");
for idx = 1:numel(batch.variants)
    v = batch.variants(idx);
    fprintf(fid, "- `%s`: %s\n", v.name, v.notes);
end

fprintf(fid, "\n## Batch Summary\n\n");
fprintf(fid, "```text\n");
dispText = evalc("disp(indexTable)");
fprintf(fid, "%s", dispText);
fprintf(fid, "```\n");
end

function writeVariantReadme(resultDir, metadata)
fid = fopen(fullfile(resultDir, "VALIDATION_README.md"), "w");
if fid < 0
    warning("validate_boundary_guard_grid:ReadmeOpenFailed", ...
        "Could not write VALIDATION_README.md in %s.", resultDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

v = metadata.variant;
fprintf(fid, "# Boundary Guard Variant\n\n");
fprintf(fid, "Variant:\n\n```text\n%s\n```\n\n", v.name);
fprintf(fid, "Notes:\n\n%s\n\n", v.notes);
fprintf(fid, "Batch directory:\n\n```text\n%s\n```\n\n", metadata.batchDir);
fprintf(fid, "Primary table:\n\n```text\nsummary.csv\n```\n");
end
