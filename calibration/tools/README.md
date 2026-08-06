# tools

这里放可复用分析工具。

适合放：

```text
extract_sim_signals.m
plot_base_response.m
plot_leg_tracking.m
compare_trials.m
compute_settling_time.m
```

规则：

```text
只服务某一个 study 的脚本，先放在 studies/<study_name>/
多个 study 都会复用的函数，再放到 tools/
和某个模型强绑定的运行逻辑，放到 experiments/<experiment_name>/
```
