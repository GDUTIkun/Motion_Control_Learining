function batch = validate_force_limit_grid_9cases(bestResultDir, variants)
%VALIDATE_FORCE_LIMIT_GRID_9CASES Test controlled force-limit scaling.
%
% This batch answers whether the 10 N pressure failures are caused by too much
% upper-layer Fx/Fz authority during recovery. It keeps the round-2 best Q/R
% and varies limit scales without changing startup.m.
%
% Usage:
%   batch = validate_force_limit_grid_9cases;
%   batch = validate_force_limit_grid_9cases("D:\...\single_wheel_leg_lqr_qr_round2\20260806_224811");

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
    error("validate_force_limit_grid_9cases:MissingBestParams", ...
        "Could not find best_params.mat in: %s", bestResultDir);
end

loaded = load(bestParamFile, "bestParams", "bestObjective");
if ~isfield(loaded, "bestParams")
    error("validate_force_limit_grid_9cases:InvalidBestParams", ...
        "best_params.mat does not contain bestParams.");
end

addpath(studyDir);
cases = disturbance_cases();

batchStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
batchDir = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_lqr_disturbance_response", "force_limit_grid_" + batchStamp);
if ~isfolder(batchDir)
    mkdir(batchDir);
end

batch = struct();
batch.batchDir = string(batchDir);
batch.bestResultDir = string(bestResultDir);
batch.bestObjective = loaded.bestObjective;
batch.variants = variants;
batch.results = repmat(struct("variant", [], "resultDir", "", "summary", []), ...
    numel(variants), 1);

for idx = 1:numel(variants)
    variant = variants(idx);
    limitParams = struct( ...
        "tauScale", variant.tauScale, ...
        "forceScale", variant.forceScale, ...
        "momentScale", variant.momentScale);

    fprintf("\nRunning force-limit variant %d/%d: %s\n", ...
        idx, numel(variants), variant.name);
    fprintf("  tauScale    = %.4g\n", variant.tauScale);
    fprintf("  forceScale  = %.4g\n", variant.forceScale);
    fprintf("  momentScale = %.4g\n", variant.momentScale);

    out = run_cases(cases, loaded.bestParams, limitParams);
    metadata = makeMetadata(bestResultDir, loaded, variant, limitParams, batchDir);
    save(fullfile(out.resultDir, "validation_metadata.mat"), "metadata");
    writeVariantReadme(out.resultDir, metadata);

    batch.results(idx).variant = variant;
    batch.results(idx).resultDir = string(out.resultDir);
    batch.results(idx).summary = out.summary;
end

indexTable = makeIndexTable(batch.results);
writetable(indexTable, fullfile(batchDir, "force_limit_grid_index.csv"));
save(fullfile(batchDir, "force_limit_grid_batch.mat"), "batch", "indexTable");
writeBatchReadme(batchDir, batch, indexTable);

fprintf("\nForce-limit grid batch saved to:\n%s\n", batchDir);
disp(indexTable);
end

function variants = defaultVariants()
variants = [
    variantDef("force_0p8_tau_1p0_moment_1p0", 1.0, 0.8, 1.0, ...
        "Only Fx/Fz force limits are reduced mildly.")
    variantDef("force_0p6_tau_1p0_moment_1p0", 1.0, 0.6, 1.0, ...
        "Only Fx/Fz force limits are reduced moderately.")
    variantDef("force_0p4_tau_1p0_moment_1p0", 1.0, 0.4, 1.0, ...
        "Only Fx/Fz force limits are reduced aggressively.")
    variantDef("force_0p6_tau_0p8_moment_1p0", 0.8, 0.6, 1.0, ...
        "Fx/Fz are reduced and joint torque limits are reduced mildly; pitch moment limit unchanged.")
    ];
end

function v = variantDef(name, tauScale, forceScale, momentScale, notes)
v = struct();
v.name = string(name);
v.tauScale = double(tauScale);
v.forceScale = double(forceScale);
v.momentScale = double(momentScale);
v.notes = string(notes);
end

function metadata = makeMetadata(bestResultDir, loaded, variant, limitParams, batchDir)
metadata = struct();
metadata.validationType = "force_limit_grid_full_9case";
metadata.bestResultDir = string(bestResultDir);
metadata.bestObjective = loaded.bestObjective;
metadata.variant = variant;
metadata.limitParams = limitParams;
metadata.batchDir = string(batchDir);
metadata.createdAt = datetime("now");
end

function indexTable = makeIndexTable(results)
n = numel(results);
variant = strings(n, 1);
resultDir = strings(n, 1);
tauScale = zeros(n, 1);
forceScale = zeros(n, 1);
momentScale = zeros(n, 1);
stableCount = zeros(n, 1);
failed10NCount = zeros(n, 1);
maxFinalAbsTheta5N = zeros(n, 1);
maxAbsX5N = zeros(n, 1);
maxAbsTheta10N = zeros(n, 1);
maxFinalAbsTheta10N = zeros(n, 1);
maxTauSaturation10N = zeros(n, 1);
maxPulseULqr10N = zeros(n, 1);

for idx = 1:n
    s = results(idx).summary;
    v = results(idx).variant;
    variant(idx) = v.name;
    resultDir(idx) = results(idx).resultDir;
    tauScale(idx) = v.tauScale;
    forceScale(idx) = v.forceScale;
    momentScale(idx) = v.momentScale;
    stableCount(idx) = sum(s.stable);

    is5N = s.pulseAmplitudeN == 5;
    is10N = s.pulseAmplitudeN == 10;
    failed10NCount(idx) = sum(~s.stable & is10N);
    maxFinalAbsTheta5N(idx) = max(abs(s.finalThetaDeg(is5N)));
    maxAbsX5N(idx) = max(abs(s.finalX(is5N)));
    maxAbsTheta10N(idx) = max(s.maxAbsThetaDeg(is10N));
    maxFinalAbsTheta10N(idx) = max(abs(s.finalThetaDeg(is10N)));
    maxTauSaturation10N(idx) = max(s.tauSaturationRatio(is10N));
    maxPulseULqr10N(idx) = max(s.pulseMaxAbsULqr(is10N));
end

indexTable = table(variant, tauScale, forceScale, momentScale, ...
    stableCount, failed10NCount, maxFinalAbsTheta5N, maxAbsX5N, ...
    maxAbsTheta10N, maxFinalAbsTheta10N, maxTauSaturation10N, ...
    maxPulseULqr10N, resultDir);
end

function writeBatchReadme(batchDir, batch, indexTable)
fid = fopen(fullfile(batchDir, "README.md"), "w");
if fid < 0
    warning("validate_force_limit_grid_9cases:ReadmeOpenFailed", ...
        "Could not write README.md in %s.", batchDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Force-Limit Grid 9-Case Batch\n\n");
fprintf(fid, "Source best QR:\n\n```text\n%s\n```\n\n", batch.bestResultDir);
fprintf(fid, "This batch tests whether 10 N failures are aggravated by excessive Fx/Fz authority.\n\n");
fprintf(fid, "Index table:\n\n```text\nforce_limit_grid_index.csv\n```\n\n");
fprintf(fid, "## Variants\n\n");
for idx = 1:numel(batch.variants)
    v = batch.variants(idx);
    fprintf(fid, "- `%s`: tau %.4g, force %.4g, moment %.4g. %s\n", ...
        v.name, v.tauScale, v.forceScale, v.momentScale, v.notes);
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
    warning("validate_force_limit_grid_9cases:ReadmeOpenFailed", ...
        "Could not write VALIDATION_README.md in %s.", resultDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

v = metadata.variant;
fprintf(fid, "# Force-Limit Grid 9-Case Validation\n\n");
fprintf(fid, "Source best QR:\n\n```text\n%s\n```\n\n", metadata.bestResultDir);
fprintf(fid, "Batch directory:\n\n```text\n%s\n```\n\n", metadata.batchDir);
fprintf(fid, "Variant:\n\n```text\n%s\n```\n\n", v.name);
fprintf(fid, "Settings:\n\n```text\ntauScale    = %.6g\nforceScale  = %.6g\nmomentScale = %.6g\n```\n\n", ...
    v.tauScale, v.forceScale, v.momentScale);
fprintf(fid, "Notes:\n\n%s\n\n", v.notes);
fprintf(fid, "Primary table:\n\n```text\nsummary.csv\n```\n");
end
