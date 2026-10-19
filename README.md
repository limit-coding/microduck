# Microduck 统一工程仓库

这个仓库把 XGO-Duck 的硬件、强化学习与两个运行时放在同一个工作区，方便团队和 WorkBuddy 在一次克隆中查看、修改和提交跨模块的改动。

| 目录 | 内容 | 原仓库 |
| --- | --- | --- |
| [`xgoduck_hardware/`](xgoduck_hardware/) | 机械结构、PCB、BOM、装配资料 | [LuwuDynamics/xgoduck_hardware](https://github.com/LuwuDynamics/xgoduck_hardware) |
| [`xgoduck_rl/`](xgoduck_rl/) | 仿真、强化学习、模型与测试 | [LuwuDynamics/xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl) |
| [`xgoduck_runtime_arduino/`](xgoduck_runtime_arduino/) | Arduino UNO Q 运行时 | [LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino) |
| [`xgoduck_runtime_mosaico/`](xgoduck_runtime_mosaico/) | Mosaico/ESP-IDF 运行时 | [LuwuDynamics/xgoduck_runtime_mosaico](https://github.com/LuwuDynamics/xgoduck_runtime_mosaico) |

## 开始协作

```bash
git clone https://github.com/limit-coding/microduck.git
cd microduck
```

先阅读所涉及目录的 README，再在分支上修改并提交 Pull Request。跨模块改动可以放在同一个 Pull Request 中。这个统一仓库中的代码是完整文件，不需要初始化子模块。

各目录是从原仓库默认分支导入的快照；导入日期、精确提交和来源见 [`UPSTREAMS.json`](UPSTREAMS.json)。今后这里的改动不会自动回写原仓库；从原仓库同步新版本时，应先核对差异，再更新该文件并说明合并内容。

## 许可与来源

请保留各目录中的原有许可和版权声明。`xgoduck_rl/` 的代码带有 Apache-2.0 许可；模型和设计文件的授权需要分别核对。其余三个原仓库在本次导入时没有根目录许可文件。硬件仓库的 [许可说明](xgoduck_hardware/LICENSING.md) 和 [来源说明](xgoduck_hardware/UPSTREAM.md) 记录了仍待确认的范围。这个仓库不声明统一许可；使用或再分发前，请确认相应组件的授权。
