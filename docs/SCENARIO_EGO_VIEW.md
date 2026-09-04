# V0.6.12.8.2.2.85：统一主车、车道安全控制和独立跟随视角

主车是 CARLA 中的仿真参考车辆，用于预警场景和本车状态输入。**不与真实 OBU 的身份、定位或速度绑定**。本版不新增 FCW/BSD 等算法，也不修改已有 `category`、`event_sort`、MQTT 路由或 RSM/RSI 编码。

## 运行

1. 启动 CARLA 0.9.15，按原方式运行 `main.py`，等待路侧背景学习 READY。
2. 在 Ubuntu 桌面另开终端，选择一个场景：

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_avw.py --ego-view
```

其他场景同样支持新参数，**每次只运行一条**：

```bash
python3.7 tools/scenario_vrucw.py --ego-view
python3.7 tools/scenario_hlw.py --ego-view
python3.7 tools/scenario_slw.py --ego-view --ego-speed-kmh 55
```

`main.py` 原来的相机模型等参数保持不变。通常不要再传 `--test-ego-speed-kmh`，因为它会覆盖主车实测速度，仅用于报文台架测试。

## 场景内容

| 场景 | 主车 | 目标 | 保留的事件映射 |
| --- | --- | --- | --- |
| VRUCW | 默认目标18 km/h | 12名行人，主车检测到同车道前方行人后制动 | VRUCW / 10 |
| HLW | 默认目标18 km/h | 每条入口车道各1个道路障碍物，主车在障碍物前制动 | HLW / 8，event_type=37 |
| AVW | 默认目标18 km/h，从目标同车道后方45–65m生成 | 1辆静止车辆，主车在目标前制动 | AVW / 6 |
| SLW | 默认期望55 km/h | 无需另造超速目标车 | SLW / 9 |

`spawn_multiclass_targets.py` 的 custom 模式也带一辆主车。`.85`主车不再交给 Traffic Manager，而是每约0.1秒跟随当前驾驶车道的CARLA航点；`--tm-port`仅为旧命令兼容保留。主车遇红/黄灯、同车道车辆、行人或场景障碍物会主动制动，车道偏差过大也会停车。SLW 的 Flag=2 要求实际速度超过限速；低于或等于限速为 Flag=1。

正常控制日志示例：

```text
[EGO CONTROL] mode=lane_follow speed=17.8km/h lane_error=0.12m steer=0.03 hazard=None clearance=-
[EGO CONTROL] mode=hazard_stop speed=11.2km/h lane_error=0.09m steer=0.01 hazard=123 clearance=7.4m
```

若出现`lane_departure_stop`，主车会停车而不是继续横切车道；请保留该段日志用于调整当前地图的控制参数。

若只想先看静态场景：

```bash
python3.7 tools/scenario_avw.py --ego-view --ego-speed-kmh 0
```

本版主车不循环瞬移回起点、不强制保证每次都碰到危险目标。主车驶离路口后，按 Ctrl+C 停止场景，再启动即可重新布置。行人、障碍物仍沿用原来的路口分布及道路存在性触发，不宣称已增加逐车碰撞预测或针对主车的前向距离过滤。

## 独立窗口

也可以不使用场景的 `--ego-view`，单独启动一个长期保留的窗口：

```bash
cd ~/RoadsideStation
python3.7 tools/ego_view.py
```

可以在场景之前启动：没主车时显示等待，有主车时自动挂接相机；切换场景后会重新查找主车。

| 按键 | 作用 |
| --- | --- |
| 1 | 车后上方跟随，默认 |
| 2 | 驾驶位附近视角 |
| 3 | 主车上方俯视 |
| V | 循环切换 |
| Q / Esc / 关闭按钮 | 仅关闭窗口，不停止场景、main.py 或 CARLA |

窗口显示主车 actor ID、实际车速、视角。固定使用显示专用 RGB 相机，不接入检测算法，不改变 CARLA spectator、世界同步设置，也不调用 `world.tick()`。

低负载示例：

```bash
python3.7 tools/ego_view.py --width 640 --height 360 --fps 15
```

默认960×540、20 FPS，会增加一台 RGB 相机的渲染开销。需要有桌面显示及项目既有的 `opencv-python`，不要用 headless OpenCV 包。`-RenderOffScreen` 可以，但 CARLA 的 `no_rendering_mode` 会使相机无有效图像。世界处于同步模式时，必须由原有主时钟客户端推进仿真；本窗口不抢占时钟。缺少帧时窗口显示等待，不长期展示旧帧。

## 状态与清理

`config/roadside.yaml` 中 `v2x_events.test_ego_role` 默认为 `rsu_test_ego`，场景、窗口和 main.py 共用。若使用场景或窗口的 `--ego-role` 覆盖参数，必须同时修改 main.py 所读配置中的角色名。`--config` 也应指向同一份配置。

- 只选择明确角色名；不随意把环境交通认作主车。
- 已有一辆同角色主车时复用，不改其控制或速度；原创建者负责删除。要换场景速度，先退出原场景。
- 多辆同角色主车会明确报错，避免把一辆车的速度用于另一辆车的窗口。建议顺序启动场景，避免同时创建的竞争。
- 场景 Ctrl+C/SIGTERM 清理自己创建的车辆、行人、控制器和附带窗口；不会删除复用主车或其他交通。
- 独立启动的窗口只销毁自己的相机，主车消失后等待下一辆。

main.py 在事件侧接入主车位置、速度、航向及车身范围，以车身范围过滤主车自身的车辆/未知障碍物检测，避免主车停车产生自己的 AVW/HLW。**原始融合列表和 RSM 仍保持路侧感知输出**，不把主车真值注入目标列表，也不从 RSM 删除已检测到的主车。

该自身过滤是空间近似：车身范围内过近目标可能被过滤，车身之外的漂移碎片可能漏过滤。行人/非机动车不参与自身过滤。后续可根据实测日志改进，不等价于已完成稳定的主车轨迹 ID 关联。

安全制动同样属于仿真场景保护层，不是正式FCW/VRU算法输出，不改变现有`category`、`event_sort`、ObjectList、RSM或RSI协议内容。

## 验证

离线回归覆盖：主车角色选择、重复/消失检测、旋转车身坐标、实测速度、AVW/HLW自身排除、相邻车辆/行人保留、目标列表不变、创建/复用清理、相机切换后的旧帧丢弃及异常清理。

```bash
python3.7 -m unittest discover -s tests
```

开发环境没有运行中的 CARLA，未进行真实渲染、道路行驶或 RSU/OBU 联调。Ubuntu 验收时检查：

1. AVW 场景出现两辆车，窗口跟随蓝色主车；停车目标经过感知确认后产生 AVW。
2. 1/2/3 切换正常，原路侧相机视角不变。
3. 关闭窗口后 main.py 和场景继续运行。
4. 用独立窗口切换场景，窗口能够重新跟随新主车。
5. SLW 日志中 `[V2X EGO]` 是实际车速，Flag 随限速比较变化。
6. Ctrl+C 后只删除本次场景创建的对象；不存在残留附带相机。
