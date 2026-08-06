function figHandles = plot_responses(out)
%PLOT_RESPONSES Plot base, LQR command, torque, and leg tracking responses.

if nargin < 1 || isempty(out)
    [file, path] = uigetfile("*.mat", "Select study_results.mat");
    if isequal(file, 0)
        figHandles = gobjects(0);
        return;
    end
    loaded = load(fullfile(path, file), "out");
    out = loaded.out;
end

figHandles = gobjects(numel(out.cases), 4);

for idx = 1:numel(out.cases)
    caseResult = out.cases(idx);
    sig = caseResult.signals;
    caseName = char(caseResult.case.name);

    figHandles(idx, 1) = plotBase(sig, caseName);
    figHandles(idx, 2) = plotLqr(sig, caseName);
    figHandles(idx, 3) = plotTau(sig, caseName);
    figHandles(idx, 4) = plotLeg(sig, caseName);
end
end

function h = plotBase(sig, caseName)
t = sig.time;
X = sig.X;
h = figure("Name", caseName + " base");
tiledlayout(3, 2);
names = ["xB", "zB", "thetaB", "dxB", "dzB", "dthetaB"];
for idx = 1:6
    nexttile;
    plot(t, X(:, idx), "LineWidth", 1.1);
    grid on;
    title(names(idx));
    xlabel("t [s]");
end
end

function h = plotLqr(sig, caseName)
t = sig.uLqr.time;
u = sig.uLqr.data;
h = figure("Name", caseName + " LQR command");
plot(t, u, "LineWidth", 1.1);
grid on;
legend("FHx_ext", "FHz_ext", "MBy_des", "Location", "best");
xlabel("t [s]");
title(caseName + " LQR command");
end

function h = plotTau(sig, caseName)
t = sig.tau.time;
tau = sig.tau.data;
h = figure("Name", caseName + " tau");
plot(t, tau, "LineWidth", 1.1);
grid on;
legend("tau_h", "tau_k", "tau_w", "Location", "best");
xlabel("t [s]");
title(caseName + " joint torques");
end

function h = plotLeg(sig, caseName)
t = sig.time;
h = figure("Name", caseName + " leg tracking");
tiledlayout(2, 2);

nexttile;
plot(t, sig.qRel(:, 1:2), "LineWidth", 1.1);
hold on;
plot(t, sig.qRef(:, 1:2), "--", "LineWidth", 1.1);
grid on;
legend("qh", "qk", "qh ref", "qk ref", "Location", "best");
title("Hip/knee position");

nexttile;
plot(t, sig.dqRel(:, 1:2), "LineWidth", 1.1);
hold on;
plot(t, sig.dqRef(:, 1:2), "--", "LineWidth", 1.1);
grid on;
legend("dqh", "dqk", "dqh ref", "dqk ref", "Location", "best");
title("Hip/knee velocity");

nexttile;
plot(t, sig.qRel(:, 3), "LineWidth", 1.1);
hold on;
plot(t, sig.qRef(:, 3), "--", "LineWidth", 1.1);
grid on;
legend("qw", "qw ref", "Location", "best");
title("Wheel position");

nexttile;
plot(t, sig.dqRel(:, 3), "LineWidth", 1.1);
hold on;
plot(t, sig.dqRef(:, 3), "--", "LineWidth", 1.1);
grid on;
legend("dqw", "dqw ref", "Location", "best");
title("Wheel velocity");
end
