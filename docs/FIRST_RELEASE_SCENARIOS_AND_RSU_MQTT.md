# 第一版：主程序常驻、按需启动预警场景、RSM发送至RSU

适用版本：`V0.6.12.8.2.2.88`。主车、车道安全控制与跟随窗口详见 [SCENARIO_EGO_VIEW.md](SCENARIO_EGO_VIEW.md)。

## 1. 运行方式

```text
CARLA（一直运行）
  |
main.py（只启动一次：感知、融合、事件判断、RSM -> MQTT -> RSU）
  |
  +-- scenario_vrucw.py（需要行人场景时启动）
  +-- scenario_hlw.py   （需要障碍物场景时启动）
  +-- scenario_avw.py   （需要异常停车场景时启动）
  +-- scenario_slw.py   （需要超速场景时启动）
```

每个场景自动创建一辆主车（已有同角色主车时复用）。场景脚本保持运行时，测试目标一直存在；按 `Ctrl+C` 后只删除该脚本生成的主车和目标，
CARLA和`main.py`不需要重启，可以接着运行另一个场景脚本。

## 2. 一次性配置MQTT和RSU坐标

编辑`config/roadside.yaml`：

```yaml
mqtt:
  enabled: true
  host: 127.0.0.1           # 与已验证的 mosquitto_pub -h localhost 一致
  port: 1883
  client_id: roadside-mec
  username:                 # 当前本地Broker不认证，保持为空
  password_env: ROADSIDE_MQTT_PASSWORD
  qos: 2
  response_topic: command///res/#
  publish_internal_outputs: false
  publish_internal_event: false

dachuan_rsu:
  enabled: true
  device_id: DC887-002047
  topic_template: command/dachuan/{device_id}/req/{uuid}/{message_type}
  reference_latitude_deg: 39.0000000   # 改成RSU/路口实测纬度
  reference_longitude_deg: 116.0000000 # 改成RSU/路口实测经度
  reference_elevation_m: 0.0
  world_x_heading_from_east_deg: 0.0
  rsm_publish_hz: 10.0
  publish_rsi_events: true
  event_source: 5
  event_priority: 1
  avw_event_type: 37
```

当前已验证的`localhost`命令不需要用户名和密码，因此不用设置密码环境变量。
只有以后Broker明确启用认证时，才填写`username`并执行：

```bash
export ROADSIDE_MQTT_PASSWORD='实际MQTT密码'
```

经纬度必须使用真实值。缺少坐标时程序会拒绝开启RSU发送，防止通过PC5广播错误位置。

## 3. 启动CARLA

```bash
cd ~/carla
./CarlaUE4.sh -RenderOffScreen
```

## 4. 启动main.py（整个测试过程只启动一次）

```bash
cd ~/RoadsideStation

python3.7 main.py \
  --camera-source detector \
  --camera-model models/yolov5n.onnx \
  2>&1 | tee v0612822280_main.log
```

不要传`--event-scenario`，默认同时启用VRUCW、HLW、AVW和SLW。

正常启动后会保留精简日志；增加 `--verbose` 才会输出原来的逐秒感知、评估和
主车状态诊断。RSI 触发时会直接打印实际发送的 topic 和完整 payload：

```text
Dachuan RSU bridge: ENABLED | RSM=10Hz continuous | RSI=event-driven
[MQTT] Connected broker=127.0.0.1:1883 qos=2
[RSI DATA] {"type":"RSI","value":{"category":"RSI",...}}
[RSU MQTT TX] type=RSI category=AVW topic=command/traffic/event/req/.../rsi
[MQTT RX] topic=command///res/.../200 payload={"reqid":"...","return_code":200}
```

如需观察背景学习状态，请用 `python3.7 main.py --verbose ...`，等待
`[BACKGROUND] Status:READY` 后再启动场景。

## 5. 按需启动场景

### 5.1 VRUCW - 道路行人

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_vrucw.py
```

场景内容：1辆主车＋12名行人分批横穿道路。

预期：`category=VRUCW`、`event_sort=10`；RSM中的行人为`ptcType=3`。

### 5.2 HLW - 道路障碍物

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_hlw.py
```

场景内容：1辆主车＋目标路口每条入口车道各1个静态障碍物；主车会在同车道障碍物前安全制动。

预期：`category=HLW`、`event_sort=8`、`event_type=37`。

### 5.3 AVW - 异常停车车辆

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_avw.py
```

场景内容：1辆主车＋1辆拉手刹的静止目标车辆；主车优先生成在目标同车道后方。路侧感知持续确认目标静止约5秒后产生
内部检测事件为 `category=AVW`、`event_sort=6`；对外消息为 RSI
`value.rtes[]`，默认使用可配置的 `eventType=37`，`eventPos` 来自该静止
车辆的感知位置。RSM中的车辆仍为`ptcType=1`、速度接近0。

### 5.4 SLW - 超速预警

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_slw.py --ego-speed-kmh 55
```

场景内容：生成统一角色名`rsu_test_ego`的主车，车道航点控制目标速度为55 km/h。
main.py读取实际速度，与配置中的40 km/h限速比较。主车可能因红灯、前车而减速；实际速度超过限速才为Flag=2。使用 `--verbose` 时可看到内部诊断：

```text
[V2X EGO] id=... Source:CARLA_EGO Speed:55.0km/h SelfExcluded:...
[V2X SLW INPUT] Speed:55.0km/h Limit:40km/h Flag:2
[INTERNAL EVENT] ... "category":"SLW" ... "event_sort":9 ... "spd_Flag":2 ...
```

## 6. 切换场景

在当前场景脚本终端按`Ctrl+C`。看到：

```text
V0.6.12.8.2.2.86 scenario actors removed.
```

然后直接启动另一个`scenario_*.py`；不要停止CARLA和main.py。

## 7. RSM参与者与RSI事件的协议边界

main.py持续将明确分类的交通参与者转换成大椽格式RSM，并发布到：

```text
command/dachuan/DC887-002047/req/{UUID}/rsm
```

- RSM只允许`ptcType=1`机动车、`ptcType=2`非机动车、`ptcType=3`行人。
- 未明确分类的障碍物不会再按尺寸猜测成机动车并混入RSM。
- 道路交通事件使用RSI的`rtes[]`，道路交通标志使用RSI的`rtss[]`。
- RSI按协议发布到`command/traffic/event/req/{UUID}/rsi`，与现场RSM Topic分开配置。
- HLW使用已经确认的`event_type=37`生成RSI RTE。
- AVW不再使用`event_sort=6`冒充协议字段，而是按配置的
  `avw_event_type=37`生成RSI RTE。
- SLW必须取得大椽确认的`signType`后才能生成RSI RTS。
- 本地`event_sort`是HMI预警枚举，不能直接当作RSI的`eventType`或`signType`。

内部 `event2hmi` JSON 默认不再发布。当前默认持续发送RSM并对已确认映射的事件发送RSI。映射未确认的事件会明确抑制，
不能把`event2hmi`原样伪装成RSM或把`event_sort`冒充标准事件类型。

## 8. 完整链路判定

1. main.py出现`[RSU MQTT TX] type=RSM`。
2. main.py收到`command///res/{UUID}/200`且`return_code=200`。
3. OBU的`rsus`或`local`中出现RSU广播的参与者数据。
4. 道路事件确认main.py出现`[RSU MQTT TX] type=RSI`。
5. 在OBU侧确认RSM参与者或RSI事件已通过PC5收到。

只看到第1步代表MEC调用了MQTT发送，不代表RSU解析或PC5链路已经成功。
