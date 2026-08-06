# configs

这里放共享配置文件，不直接放某一次实验生成的数据。

适合放：

```text
扰动工况配置
默认 LQR/QP 参数配置
指标权重配置
画图风格配置
批量扫描参数范围
```

示例文件名：

```text
default_lqr_qp_cases.m
disturbance_cases.m
plotting_style.m
metric_weights.m
```

如果某个配置只服务一个具体 study，就优先放到对应的 `studies/<study_name>/` 里。只有多个 study 都会复用时，再放到这里。
