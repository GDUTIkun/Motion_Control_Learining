function batch = validate_fhx_shaping_9cases(bestResultDir, variants)
%VALIDATE_FHX_SHAPING_9CASES Test optional FHx_ext output shaping.
%
% This runs one full 9-case validation per variant. It does not edit
% startup.m; shaping is enabled only through the lqrParams struct passed to
% run_cases.
%
% Usage:
%   batch = validate_fhx_shaping_9cases;
%   batch = validate_fhx_shaping_9cases("D:\...\single_wheel_leg_lqr_qr_round2\20260806_224811");

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
    error("validate_fhx_shaping_9cases:MissingBestParams", ...
        "Could not find best_params.mat in: %s", bestResultDir);
end

loaded = load(bestParamFile, "bestParams", "bestObjective");
if ~isfield(loaded, "bestParams")
    error("validate_fhx_shaping_9cases:InvalidBestParams", ...
        "best_params.mat does not contain bestParams.");
end

addpath(studyDir);
cases = disturbance_cases();

batchStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
batchDir = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_lqr_disturbance_response", "fhx_shaping_" + batchStamp);
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
    params = applyVariant(loaded.bestParams, variant);
    fprintf("\nRunning FHx shaping variant %d/%d: %s\n", ...
        idx, numel(variants), variant.name);
    fprintf("  Q_dx scale = %.4g\n", variant.qdxScale);
    fprintf("  filterTau  = %.4g s\n", variant.filterTau);
    fprintf("  rateLimit  = %.4g N/s\n", variant.rateLimit);

    out = run_cases(cases, params);
    metadata = makeMetadata(bestResultDir, loaded, variant, params, batchDir);
    save(fullfile(out.resultDir, "validation_metadata.mat"), "metadata");
    writeVariantReadme(out.resultDir, metadata);

    batch.results(idx).variant = variant;
    batch.results(idx).resultDir = string(out.resultDir);
    batch.results(idx).summary = out.summary;
end

indexTable = makeIndexTable(batch.results);
writetable(indexTable, fullfile(batchDir, "fhx_shaping_index.csv"));
save(fullfile(batchDir, "fhx_shaping_batch.mat"), "batch", "indexTable");
writeBatchReadme(batchDir, batch, indexTable);

fprintf("\nFHx shaping batch saved to:\n%s\n", batchDir);
disp(indexTable);
end

function variants = defaultVariants()
variants = [
    variantDef("fhx_filter_30ms", NaN, 0.030, inf, ...
        "Round-2 best with first-order filtering on FHx_ext only.")
    variantDef("fhx_rate_2000", NaN, 0, 2000, ...
        "Round-2 best with FHx_ext rate limit of 2000 N/s.")
    variantDef("qdx_2p5_fhx_filter_30ms", 2.5, 0.030, inf, ...
        "Round-2 best with Q_dx reduced to 2.5x and FHx_ext filtering.")
    ];
end

function v = variantDef(name, qdxScale, filterTau, rateLimit, notes)
v = struct();
v.name = string(name);
v.qdxScale = double(qdxScale);
v.filterTau = double(filterTau);
v.rateLimit = double(rateLimit);
v.notes = string(notes);
end

function params = applyVariant(params, variant)
baseQDiag = [25; 80; 120; 8; 16; 10];

if isfinite(variant.qdxScale)
    params.QDiag = diag(params.Q);
    params.QDiag(4) = baseQDiag(4) * variant.qdxScale;
    params.Q = diag(params.QDiag);
    params.scale.qdx = variant.qdxScale;
    params.log_s_qdx = log10(variant.qdxScale);
end

params.commandShaping = struct();
params.commandShaping.enabled = true;
params.commandShaping.channels = [true; false; false];
params.commandShaping.filterTau = variant.filterTau;
params.commandShaping.rateLimit = variant.rateLimit;
end

function metadata = makeMetadata(bestResultDir, loaded, variant, params, batchDir)
metadata = struct();
metadata.validationType = "fhx_shaping_full_9case";
metadata.bestResultDir = string(bestResultDir);
metadata.bestObjective = loaded.bestObjective;
metadata.variant = variant;
metadata.params = params;
metadata.batchDir = string(batchDir);
metadata.createdAt = datetime("now");
end

function indexTable = makeIndexTable(results)
n = numel(results);
variant = strings(n, 1);
resultDir = strings(n, 1);
qdxScale = NaN(n, 1);
filterTau = zeros(n, 1);
rateLimit = zeros(n, 1);
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
    qdxScale(idx) = v.qdxScale;
    filterTau(idx) = v.filterTau;
    rateLimit(idx) = v.rateLimit;
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

indexTable = table(variant, qdxScale, filterTau, rateLimit, ...
    stableCount, failed10NCount, maxFinalAbsTheta5N, maxAbsX5N, ...
    maxAbsTheta10N, maxFinalAbsTheta10N, maxTauSaturation10N, ...
    maxPulseULqr10N, resultDir);
end

function writeBatchReadme(batchDir, batch, indexTable)
fid = fopen(fullfile(batchDir, "README.md"), "w");
if fid < 0
    warning("validate_fhx_shaping_9cases:ReadmeOpenFailed", ...
        "Could not write README.md in %s.", batchDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# FHx Output Shaping 9-Case Batch\n\n");
fprintf(fid, "Source best QR:\n\n```text\n%s\n```\n\n", batch.bestResultDir);
fprintf(fid, "This batch tests optional output shaping on `FHx_ext` only.\n\n");
fprintf(fid, "Index table:\n\n```text\nfhx_shaping_index.csv\n```\n\n");
fprintf(fid, "## Variants\n\n");
for idx = 1:numel(batch.variants)
    v = batch.variants(idx);
    fprintf(fid, "- `%s`: Q_dx scale %.4g, filterTau %.4g s, rateLimit %.4g N/s. %s\n", ...
        v.name, v.qdxScale, v.filterTau, v.rateLimit, v.notes);
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
    warning("validate_fhx_shaping_9cases:ReadmeOpenFailed", ...
        "Could not write VALIDATION_README.md in %s.", resultDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

v = metadata.variant;
fprintf(fid, "# FHx Output Shaping 9-Case Validation\n\n");
fprintf(fid, "Source best QR:\n\n```text\n%s\n```\n\n", metadata.bestResultDir);
fprintf(fid, "Batch directory:\n\n```text\n%s\n```\n\n", metadata.batchDir);
fprintf(fid, "Variant:\n\n```text\n%s\n```\n\n", v.name);
fprintf(fid, "Settings:\n\n```text\nQ_dx scale = %.6g\nfilterTau  = %.6g s\nrateLimit  = %.6g N/s\n```\n\n", ...
    v.qdxScale, v.filterTau, v.rateLimit);
fprintf(fid, "Notes:\n\n%s\n\n", v.notes);
fprintf(fid, "Primary table:\n\n```text\nsummary.csv\n```\n");
end
