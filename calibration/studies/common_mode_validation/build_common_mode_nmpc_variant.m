function info = build_common_mode_nmpc_variant(name)
%BUILD_COMMON_MODE_NMPC_VARIANT Build one validation solver in isolation.

name = string(name);
validNames = ["r1_0p5x", "r1_2x", "r2_0p5x", "r2_2x"];
assert(isscalar(name) && any(name == validNames), ...
    "Unknown common-mode NMPC validation variant: %s", name);

studyDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(fileparts(fileparts(studyDir)));
modelDir = fullfile(repoRoot, "model", "simulate", "two_legs");
addpath(modelDir);
evalin("base", "run('" + replace(string(fullfile(modelDir, "startup.m")), ...
    "'", "''") + "');");
baseNmpc = evalin("base", "baseNmpc");

config = baseNmpc;
if endsWith(name, "0p5x")
    scale = 0.5;
else
    scale = 2.0;
end
if startsWith(name, "r1")
    config.R1 = scale * baseNmpc.R1;
else
    config.R2 = scale * baseNmpc.R2;
end
config.W_e = config.Q;
config.solverName = "base_wheel_8state_nmpc_val_" + name;
config.sfunName = "acados_solver_sfunction_" + config.solverName;
config.buildTag = name;
config.generatedDir = fullfile(modelDir, "generated", ...
    "base_wheel_8state_nmpc_validation", name);
config.available = false;
config.referenceSize = 14*config.N + 8;

info = build_base_nmpc_solver(false, false, config);
end
