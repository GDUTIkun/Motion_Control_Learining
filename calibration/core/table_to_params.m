function params = table_to_params(x, cfg)
%TABLE_TO_PARAMS Convert a bayesopt table row to a plain parameter struct.

if nargin < 2
    cfg = struct();
end

if istable(x)
    if height(x) ~= 1
        error("table_to_params:ExpectedSingleRow", ...
            "Expected a single-row table, got %d rows.", height(x));
    end

    params = struct();
    names = x.Properties.VariableNames;
    for idx = 1:numel(names)
        name = names{idx};
        value = x.(name);
        params.(name) = normalizeValue(value);
    end
elseif isstruct(x)
    params = x;
else
    error("table_to_params:UnsupportedInput", ...
        "Expected a table row or struct.");
end

if isfield(cfg, "mapParamsFcn") && ~isempty(cfg.mapParamsFcn)
    params = cfg.mapParamsFcn(params, cfg);
    if ~isstruct(params)
        error("table_to_params:InvalidMappedParams", ...
            "cfg.mapParamsFcn must return a struct.");
    end
end
end

function value = normalizeValue(value)
if iscell(value) && isscalar(value)
    value = value{1};
elseif iscategorical(value) && isscalar(value)
    value = char(value);
elseif isstring(value) && isscalar(value)
    value = char(value);
elseif isnumeric(value) || islogical(value)
    if isscalar(value)
        value = value(1);
    end
end
end
