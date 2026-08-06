# studies

这里放具体分析课题。一个 study 应该回答一个明确问题。

命名建议：

```text
YYYY_MM_topic/
```

示例：

```text
2026_08_lqr_disturbance_response/
2026_08_qr_sweep/
2026_08_contact_sensitivity/
```

一个 study 里通常可以放：

```text
README.md
run_cases.m
plot_responses.m
extract_metrics.m
notes.md
```

如果代码变得通用，再沉淀到 `tools/` 或 `experiments/`。
