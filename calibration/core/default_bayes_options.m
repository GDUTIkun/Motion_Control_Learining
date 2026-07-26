function options = default_bayes_options(cfg)
%DEFAULT_BAYES_OPTIONS Build bayesopt name-value options from a config.

options = {
    "MaxObjectiveEvaluations", cfg.maxObjectiveEvaluations
    "IsObjectiveDeterministic", logical(cfg.isDeterministic)
    "UseParallel", logical(cfg.useParallel)
    "AcquisitionFunctionName", getDefault(cfg, "acquisitionFunctionName", "expected-improvement-plus")
    "Verbose", getDefault(cfg, "verbose", 1)
    "PlotFcn", getDefault(cfg, "plotFcn", [])
    };

options = reshape(options.', 1, []);

if isfield(cfg, "bayesoptOptions") && ~isempty(cfg.bayesoptOptions)
    options = mergeOptions(options, cfg.bayesoptOptions);
end
end

function value = getDefault(cfg, fieldName, defaultValue)
if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
    value = cfg.(fieldName);
else
    value = defaultValue;
end
end

function options = mergeOptions(options, overrides)
if isstruct(overrides)
    names = fieldnames(overrides);
    extra = cell(1, 2 * numel(names));
    for idx = 1:numel(names)
        extra{2 * idx - 1} = names{idx};
        extra{2 * idx} = overrides.(names{idx});
    end
elseif iscell(overrides)
    extra = overrides;
else
    error("default_bayes_options:InvalidOverrides", ...
        "cfg.bayesoptOptions must be a struct or name-value cell array.");
end

if mod(numel(extra), 2) ~= 0
    error("default_bayes_options:InvalidOverrides", ...
        "cfg.bayesoptOptions must contain name-value pairs.");
end

for idx = 1:2:numel(extra)
    name = string(extra{idx});
    match = find(strcmpi(string(options(1:2:end)), name), 1);
    if isempty(match)
        options(end + 1:end + 2) = extra(idx:idx + 1);
    else
        options{2 * match} = extra{idx + 1};
    end
end
end
