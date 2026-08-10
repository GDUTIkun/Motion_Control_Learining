function summary = finalize_common_mode_validation()
%FINALIZE_COMMON_MODE_VALIDATION Merge completed cases and boundary failures.

studyDir = fileparts(mfilename("fullpath"));
dataDir = fullfile(studyDir, "data");
files = dir(fullfile(dataDir, "*.mat"));
rows = [];
baseline = [];
for idx = 1:numel(files)
    stored = load(fullfile(files(idx).folder, files(idx).name), ...
        "metrics", "baseline");
    if isfield(stored, "metrics")
        if isempty(rows)
            rows = stored.metrics;
        else
            match = find([rows.case_name] == stored.metrics.case_name, 1);
            if isempty(match)
                rows(end+1) = stored.metrics; %#ok<AGROW>
            else
                rows(match) = stored.metrics;
            end
        end
    end
    if isempty(baseline) && isfield(stored, "baseline")
        baseline = stored.baseline;
    end
end
assert(~isempty(rows), "No common-mode validation case results were found.");
if isempty(baseline)
    previous = load(fullfile(dataDir, "summary.mat"), "baseline");
    baseline = previous.baseline;
end

velocity = rows(find([rows.case_name] == "velocity_baseline", 1));
baselineConstraint = velocity;
baselineConstraint.case_name = "constraint_baseline_v0p5_a0p5";
baselineConstraint.experiment = "constraint";
baselineConstraint.parameter = "baseline";
baselineConstraint.accepted = baselineConstraint.all_finite ...
    && baselineConstraint.nmpc_status_zero_ratio == 1 ...
    && baselineConstraint.nmpc_fault_zero_ratio == 1 ...
    && baselineConstraint.qp_feasible_ratio > 0.99 ...
    && baselineConstraint.max_dynamics_residual < 1e-6 ...
    && baselineConstraint.velocity_mae_mps < 0.15 ...
    && baselineConstraint.max_pitch_rad < deg2rad(5);
rows = upsert(rows, baselineConstraint);

failedCases = [
    "constraint_baseline_v0p75_a0p75", "baseline", 0.75, ...
        "Simulation did not complete: variable-step collapse after demand increase."
    "constraint_low_mu_v0p75_a0p75", "QP mu=0.20", 0.75, ...
        "Simulation did not complete: variable-step collapse after demand increase."
    "constraint_low_tau_v0p75_a0p75", "tauMax=0.25x", 0.75, ...
        "Simulation did not complete: variable-step collapse after demand increase."
    "constraint_baseline_v1_a1", "baseline", 1.0, ...
        "Not run: the lower 0.75 demand point already failed."
    "constraint_low_mu_v1_a1", "QP mu=0.20", 1.0, ...
        "Not run: the lower 0.75 demand point already failed."
    "constraint_low_tau_v1_a1", "tauMax=0.25x", 1.0, ...
        "Not run: the lower 0.75 demand point already failed."
];
failureRows = table('Size', [size(failedCases, 1), 5], ...
    'VariableTypes', ["string", "string", "double", "double", "string"], ...
    'VariableNames', ["case_name", "parameter", "command_v_mps", ...
        "command_a_mps2", "failure_reason"]);
for idx = 1:size(failedCases, 1)
    failed = blankMetrics(rows(1));
    failed.case_name = failedCases(idx, 1);
    failed.experiment = "constraint";
    failed.parameter = failedCases(idx, 2);
    failed.command_v_mps = str2double(failedCases(idx, 3));
    failed.command_a_mps2 = failed.command_v_mps;
    failed.error_message = failedCases(idx, 4);
    rows = upsert(rows, failed);
    failureRows(idx, :) = {failed.case_name, failed.parameter, ...
        failed.command_v_mps, failed.command_a_mps2, failed.error_message};
end
writetable(failureRows, fullfile(dataDir, "constraint_failures.csv"));

summary = struct2table(rows);
summary = sortrows(summary, ["experiment", "case_name"]);
writetable(summary, fullfile(dataDir, "summary.csv"));
save(fullfile(dataDir, "summary.mat"), "summary", "baseline");
analyze_common_mode_validation(studyDir);
end

function rows = upsert(rows, value)
match = find([rows.case_name] == value.case_name, 1);
if isempty(match)
    rows(end+1) = value;
else
    rows(match) = value;
end
end

function value = blankMetrics(template)
value = template;
names = fieldnames(value);
for idx = 1:numel(names)
    current = value.(names{idx});
    if isstring(current)
        value.(names{idx}) = "";
    elseif islogical(current)
        value.(names{idx}) = false;
    elseif isnumeric(current)
        value.(names{idx}) = NaN(size(current));
    end
end
end
