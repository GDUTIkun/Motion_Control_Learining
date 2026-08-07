function out = validate_limit_scale_9cases(limitScale, bestResultDir)
%VALIDATE_LIMIT_SCALE_9CASES Run full 9-case validation with scaled limits.
%
% Usage:
%   out = validate_limit_scale_9cases(2.0);
%   out = validate_limit_scale_9cases(2.0, "D:\...\results\single_wheel_leg_lqr_qr\20260806_203337");
%   limits = struct("tauScale", 2.0, "forceScale", 1.0, "momentScale", 1.0);
%   out = validate_limit_scale_9cases(limits, "D:\...\results\single_wheel_leg_lqr_qr\20260806_203337");
%
% If bestResultDir is omitted, the default startup Q/R is used.
% If bestResultDir is provided, best_params.mat is loaded and used.

if nargin < 1 || isempty(limitScale)
    limitScale = 2.0;
end
if nargin < 2
    bestResultDir = "";
end

studyDir = fileparts(mfilename("fullpath"));
addpath(studyDir);

lqrParams = [];
sourceLabel = "default_qr";
bestObjective = NaN;
bestParamFile = "";

if strlength(string(bestResultDir)) > 0
    bestResultDir = char(java.io.File(bestResultDir).getCanonicalPath());
    bestParamFile = fullfile(bestResultDir, "best_params.mat");
    if ~isfile(bestParamFile)
        error("validate_limit_scale_9cases:MissingBestParams", ...
            "Could not find best_params.mat in: %s", bestResultDir);
    end

    data = load(bestParamFile, "bestParams", "bestObjective");
    lqrParams = data.bestParams;
    bestObjective = data.bestObjective;
    sourceLabel = "best_qr";
end

if isstruct(limitScale)
    limitParams = limitScale;
else
    limitParams = struct( ...
        "tauScale", double(limitScale), ...
        "forceScale", double(limitScale), ...
        "momentScale", double(limitScale));
end

limitParams = normalizeLimitParams(limitParams);

cases = disturbance_cases();
out = run_cases(cases, lqrParams, limitParams);

metadata = struct();
metadata.validationType = "limit_scaled_full_9case";
metadata.sourceLabel = sourceLabel;
metadata.limitScale = limitParams;
metadata.limitParams = limitParams;
metadata.bestResultDir = string(bestResultDir);
metadata.bestParamFile = string(bestParamFile);
metadata.bestObjective = bestObjective;
metadata.createdAt = datetime("now");

save(fullfile(out.resultDir, "validation_metadata.mat"), "metadata");
writeValidationReadme(out.resultDir, metadata);

fprintf("\nLimit-scale validation saved to:\n%s\n", out.resultDir);
end

function limitParams = normalizeLimitParams(limitParams)
limitParams.tauScale = getFieldOrDefault(limitParams, "tauScale", 1.0);
limitParams.forceScale = getFieldOrDefault(limitParams, "forceScale", 1.0);
limitParams.momentScale = getFieldOrDefault(limitParams, "momentScale", 1.0);
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = double(s.(name));
else
    value = defaultValue;
end
end

function writeValidationReadme(resultDir, metadata)
fid = fopen(fullfile(resultDir, "VALIDATION_README.md"), "w");
if fid < 0
    warning("validate_limit_scale_9cases:ReadmeOpenFailed", ...
        "Could not write VALIDATION_README.md in %s.", resultDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Limit-Scale 9-Case Validation\n\n");
fprintf(fid, "This result validates the full stage-A 9-case disturbance set with scaled controller limits.\n\n");
fprintf(fid, "## Controller Source\n\n");
fprintf(fid, "```text\n%s\n```\n\n", metadata.sourceLabel);
if strlength(metadata.bestResultDir) > 0
    fprintf(fid, "Best QR source:\n\n");
    fprintf(fid, "```text\n%s\n```\n\n", metadata.bestResultDir);
    fprintf(fid, "Best objective:\n\n");
    fprintf(fid, "```text\n%.10g\n```\n\n", metadata.bestObjective);
end
fprintf(fid, "## Limit Scale\n\n");
fprintf(fid, "```text\n");
fprintf(fid, "tauScale    = %.6g\n", metadata.limitParams.tauScale);
fprintf(fid, "forceScale  = %.6g\n", metadata.limitParams.forceScale);
fprintf(fid, "momentScale = %.6g\n", metadata.limitParams.momentScale);
fprintf(fid, "```\n\n");
fprintf(fid, "Applied after each per-case startup and before simulation.\n\n");
fprintf(fid, "## Cases\n\n");
fprintf(fid, "```text\n");
fprintf(fid, "theta0 = [-10, 0, 10] deg\n");
fprintf(fid, "pulseAmplitude = [0, 5, 10] N\n");
fprintf(fid, "pulseWindow = [2.0, 2.5] s\n");
fprintf(fid, "stopTime = 5 s\n");
fprintf(fid, "```\n\n");
fprintf(fid, "Primary table:\n\n");
fprintf(fid, "```text\nsummary.csv\n```\n");
end
