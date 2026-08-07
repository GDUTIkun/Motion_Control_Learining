function out = validate_best_qr_9cases(bestResultDir)
%VALIDATE_BEST_QR_9CASES Run the full 9-case study with a saved best QR set.
%
% Usage:
%   out = validate_best_qr_9cases;
%   out = validate_best_qr_9cases("D:\...\results\single_wheel_leg_lqr_qr\20260806_203337");

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());

if nargin < 1 || strlength(string(bestResultDir)) == 0
    bestResultDir = latestBestResultDir(calibrationDir);
end

bestResultDir = char(java.io.File(bestResultDir).getCanonicalPath());
bestParamFile = fullfile(bestResultDir, "best_params.mat");
if ~isfile(bestParamFile)
    error("validate_best_qr_9cases:MissingBestParams", ...
        "Could not find best_params.mat in: %s", bestResultDir);
end

data = load(bestParamFile, "bestParams", "bestObjective");
if ~isfield(data, "bestParams")
    error("validate_best_qr_9cases:InvalidBestParams", ...
        "best_params.mat does not contain bestParams.");
end

addpath(studyDir);
cases = disturbance_cases();
out = run_cases(cases, data.bestParams);

metadata = struct();
metadata.validationType = "best_lqr_qr_full_9case";
metadata.bestResultDir = string(bestResultDir);
metadata.bestParamFile = string(bestParamFile);
metadata.bestObjective = data.bestObjective;
metadata.bestParams = data.bestParams;
metadata.createdAt = datetime("now");

save(fullfile(out.resultDir, "validation_metadata.mat"), "metadata");
writeValidationReadme(out.resultDir, metadata);

fprintf("\nValidated best QR parameters from:\n%s\n", bestResultDir);
fprintf("Validation results saved to:\n%s\n", out.resultDir);
end

function resultDir = latestBestResultDir(calibrationDir)
root = fullfile(calibrationDir, "results", "single_wheel_leg_lqr_qr");
dirs = dir(root);
dirs = dirs([dirs.isdir]);
names = string({dirs.name});
keep = names ~= "." & names ~= "..";
dirs = dirs(keep);
if isempty(dirs)
    error("validate_best_qr_9cases:NoResults", ...
        "No result directories found under: %s", root);
end

[~, idx] = max([dirs.datenum]);
resultDir = fullfile(dirs(idx).folder, dirs(idx).name);
end

function writeValidationReadme(resultDir, metadata)
fid = fopen(fullfile(resultDir, "VALIDATION_README.md"), "w");
if fid < 0
    warning("validate_best_qr_9cases:ReadmeOpenFailed", ...
        "Could not write VALIDATION_README.md in %s.", resultDir);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "# Best LQR Q/R 9-Case Validation\n\n");
fprintf(fid, "This result validates saved Bayesian-optimized LQR Q/R parameters on the full stage-A 9-case disturbance set.\n\n");
fprintf(fid, "## Source\n\n");
fprintf(fid, "```text\n%s\n```\n\n", metadata.bestResultDir);
fprintf(fid, "Best objective from training run:\n\n");
fprintf(fid, "```text\n%.10g\n```\n\n", metadata.bestObjective);
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
