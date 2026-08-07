function batch = validate_qx_qdx_rollback_9cases(bestResultDir, variants)
%VALIDATE_QX_QDX_ROLLBACK_9CASES Test lower x/dx Q weights from a best QR set.
%
% This runs one full 9-case validation per variant. It does not edit
% startup.m; each variant is applied after per-case startup through run_cases.
%
% Usage:
%   batch = validate_qx_qdx_rollback_9cases;
%   batch = validate_qx_qdx_rollback_9cases("D:\...\single_wheel_leg_lqr_qr_round2\20260806_224811");

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
    error("validate_qx_qdx_rollback_9cases:MissingBestParams", ...
        "Could not find best_params.mat in: %s", bestResultDir);
end

loaded = load(bestParamFile, "bestParams", "bestObjective");
if ~isfield(loaded, "bestParams")
    error("validate_qx_qdx_rollback_9cases:InvalidBestParams", ...
        "best_params.mat does not contain bestParams.");
end

addpath(studyDir);
cases = disturbance_cases();

batchStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
batchDir = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_lqr_disturbance_response", ...
    "qx_qdx_rollback_" + batchStamp);
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
    fprintf("\nRunning rollback variant %d/%d: %s\n", ...
        idx, numel(variants), variant.name);
    fprintf("  qx scale  = %.4g\n", variant.qxScale);
    fprintf("  qdx scale = %.4g\n", variant.qdxScale);

    out = run_cases(cases, params);
    metadata = makeMetadata(bestResultDir, loaded, variant, params, batchDir);
    save(fullfile(out.resultDir, "validation_metadata.mat"), "metadata");
    writeVariantReadme(out.resultDir, metadata);

    batch.results(idx).variant = variant;
    batch.results(idx).resultDir = string(out.resultDir);
    batch.results(idx).summary = out.summary;
end

indexTable = makeIndexTable(batch.results);
writetable(indexTable, fullfile(batchDir, "rollback_index.csv"));
save(fullfile(batchDir, "rollback_batch.mat"), "batch", "indexTable");
writeBatchReadme(batchDir, batch, indexTable);

fprintf("\nRollback batch saved to:\n%s\n", batchDir);
disp(indexTable);
end

function variants = defaultVariants()
variants = [
    variantDef("qdx_2p0", 0.60557, 2.0, ...
        "Round-2 best with Q_dx reduced from about 3.39x to 2.0x.")
    variantDef("qdx_1p0", 0.60557, 1.0, ...
        "Round-2 best with Q_dx reduced to the startup baseline.")
    variantDef("qx_0p3_qdx_1p0", 0.3, 1.0, ...
        "Round-2 best with lower x penalty and baseline dx penalty.")
    ];
end

function v = variantDef(name, qxScale, qdxScale, notes)
v = struct();
v.name = string(name);
v.qxScale = double(qxScale);
v.qdxScale = double(qdxScale);
v.notes = string(notes);
end

function params = applyVariant(params, variant)
baseQDiag = [25; 80; 120; 8; 16; 10];

params.QDiag = diag(params.Q);
params.QDiag(1) = baseQDiag(1) * variant.qxScale;
params.QDiag(4) = baseQDiag(4) * variant.qdxScale;
params.Q = diag(params.QDiag);

params.scale.qx = variant.qxScale;
params.scale.qdx = variant.qdxScale;
params.log_s_qx = log10(variant.qxScale);
params.log_s_qdx = log10(variant.qdxScale);
end

function metadata = makeMetadata(bestResultDir, loaded, variant, params, batchDir)
metadata = struct();
metadata.validationType = "qx_qdx_rollback_full_9case";
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
qxScale = zeros(n, 1);
qdxScale = zeros(n, 1);
stableCount = zeros(n, 1);
failed10NCount = zeros(n, 1);
maxFinalAbsTheta5N = zeros(n, 1);
maxAbsX5N = zeros(n, 1);
maxAbsTheta10N = zeros(n, 1);
maxFinalAbsTheta10N = zeros(n, 1);
maxTauSaturation10N = zeros(n, 1);

for idx = 1:n
    s = results(idx).summary;
    v = results(idx).variant;
    variant(idx) = v.name;
    resultDir(idx) = results(idx).resultDir;
    qxScale(idx) = v.qxScale;
    qdxScale(idx) = v.qdxScale;
    stableCount(idx) = sum(s.stable);

    is5N = s.pulseAmplitudeN == 5;
    is10N = s.pulseAmplitudeN == 10;
    failed10NCount(idx) = sum(~s.stable & is10N);
    maxFinalAbsTheta5N(idx) = max(abs(s.finalThetaDeg(is5N)));
    maxAbsX5N(idx) = max(abs(s.finalX(is5N)));
    maxAbsTheta10N(idx) = max(s.maxAbsThetaDeg(is10N));
    maxFinalAbsTheta10N(idx) = max(abs(s.finalThetaDeg(is10N)));
    maxTauSaturation10N(idx) = max(s.tauSaturationRatio(is10N));
end

indexTable = table(variant, qxScale, qdxScale, stableCount, ...
    failed10NCount, maxFinalAbsTheta5N, maxAbsX5N, maxAbsTheta10N, ...
    maxFinalAbsTheta10N, maxTauSaturation10N, resultDir);
end

function writeBatchReadme(batchDir, batch, indexTable)
fid = fopen(fullfile(batchDir, "README.md"), "w");
if fid < 0
    warning("validate_qx_qdx_rollback_9cases:ReadmeOpenFailed", ...
        "Could not write README.md in %s.", batchDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Qx/Qdx Rollback 9-Case Batch\n\n");
fprintf(fid, "Source best QR:\n\n```text\n%s\n```\n\n", batch.bestResultDir);
fprintf(fid, "This batch tests lower `Q_x` / `Q_dx` weights while keeping all other round-2 best parameters unchanged.\n\n");
fprintf(fid, "Index table:\n\n```text\nrollback_index.csv\n```\n\n");
fprintf(fid, "## Variants\n\n");
for idx = 1:numel(batch.variants)
    v = batch.variants(idx);
    fprintf(fid, "- `%s`: Q_x scale %.4g, Q_dx scale %.4g. %s\n", ...
        v.name, v.qxScale, v.qdxScale, v.notes);
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
    warning("validate_qx_qdx_rollback_9cases:ReadmeOpenFailed", ...
        "Could not write VALIDATION_README.md in %s.", resultDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

v = metadata.variant;
fprintf(fid, "# Qx/Qdx Rollback 9-Case Validation\n\n");
fprintf(fid, "Source best QR:\n\n```text\n%s\n```\n\n", metadata.bestResultDir);
fprintf(fid, "Batch directory:\n\n```text\n%s\n```\n\n", metadata.batchDir);
fprintf(fid, "Variant:\n\n```text\n%s\n```\n\n", v.name);
fprintf(fid, "Scales:\n\n```text\nQ_x  = %.6g\nQ_dx = %.6g\n```\n\n", ...
    v.qxScale, v.qdxScale);
fprintf(fid, "Notes:\n\n%s\n\n", v.notes);
fprintf(fid, "Primary table:\n\n```text\nsummary.csv\n```\n");
end
