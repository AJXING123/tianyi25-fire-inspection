# fire-inspection —— 天轶 2.5 消防巡检

天轶 2.5 人形机器人的消防轨道巡检项目：沿预设轨道巡航，在固定点位停下，
用视觉检查灭火器 / 消防设施 / 安全出口是否正常，并配合语音播报巡检状态。

## 仓库结构

```
fire-inspection/
├── fire_inspection_ws/     # ROS 2 工作空间（全部源码与文档）
│   └── README.md           # 详细文档：架构、接口、启动方式、dry_run 验证
├── models/                 # 模型权重（不入库，需手工创建并放置）
└── LICENSE
```

完整文档见 [fire_inspection_ws/README.md](fire_inspection_ws/README.md)。

## 模型权重

权重文件体积较大，**不提交到仓库**（见 `.gitignore`）。首次部署时把权重放到：

```
models/fire_3cls_v2.pt
```

路径由 `fire_inspection_ws/src/fire_inspection_vision/config/vision.yaml` 的
`model_path` 指定。权重缺失时视觉结果为 `NO_MODEL`，不会把点位判为正常。

## 平台依赖

`fire_inspection_voice` 依赖 `interaction_msgs`，`fire_inspection_head` 依赖
`ros2_bridge_msgs`。两者都来自实机平台环境 `/home/nvidia/xos`，`rosdep` 无法解析，
编译与运行前必须：

```bash
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash
```

## 当前状态

导航（dry_run）、视觉检查、头部扫视已实现；语音与编排层仍为骨架。
详见 [fire_inspection_ws/README.md](fire_inspection_ws/README.md) 的状态表。

## 许可证

[MIT](LICENSE)
