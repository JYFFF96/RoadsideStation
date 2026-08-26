# 第一版：主程序常驻、按需启动预警场景、RSM发送至RSU

适用版本：`V0.6.12.8.2.2.81`。

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

场景脚本保持运行时，测试目标一直存在；按 `Ctrl+C` 后只删除该脚本生成的目标，
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

dachuan_rsu:
  enabled: true
  device_id: DC887-002047
  topic_template: command/dachuan/{device_id}/req/{uuid}/{message_type}
  reference_latitude_deg: 39.0000000   # 改成RSU/路口实测纬度
  reference_longitude_deg: 116.0000000 # 改成RSU/路口实测经度
  reference_elevation_m: 0.0
  world_x_heading_from_east_deg: 0.0
  rsm_publish_hz: 10.0
  publish_rsi_events: false
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

正常启动后应看到：

```text
Dachuan RSU bridge: ENABLED | RSM=10Hz continuous | RSI=disabled
[MQTT] Connected broker=127.0.0.1:1883 qos=2
[RSU MQTT TX] type=RSM topic=command/dachuan/DC887-002047/req/.../rsm participants=...
[MQTT RX] topic=command///res/.../200 payload={"reqid":"...","return_code":200}
```

等待竖向的`[BACKGROUND] Status:READY`后再启动场景。

## 5. 按需启动场景

### 5.1 VRUCW - 道路行人

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_vrucw.py
```

场景内容：12名行人分批横穿道路。

预期：`category=VRUCW`、`event_sort=10`；RSM中的行人为`ptcType=3`。

### 5.2 HLW - 道路障碍物

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_hlw.py
```

场景内容：道路内生成6个静态障碍物。

预期：`category=HLW`、`event_sort=8`、`event_type=37`。

### 5.3 AVW - 异常停车车辆

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_avw.py
```

场景内容：道路内生成1辆拉手刹的静止车辆。持续约5秒后产生
`category=AVW`、`event_sort=6`；RSM中的车辆为`ptcType=1`、速度接近0。

### 5.4 SLW - 超速预警

```bash
cd ~/RoadsideStation
python3.7 tools/scenario_slw.py --ego-speed-kmh 55
```

场景内容：生成角色名为`rsu_test_speeding_vehicle`的测试车辆并维持约55 km/h。
main.py自动读取车辆速度，与配置中的40 km/h限速比较。预期：

```text
[V2X SLW INPUT] Source:CARLA_SCENARIO Speed:55.0km/h Limit:40km/h Flag:2
[V2X EVENT] ... "category":"SLW" ... "event_sort":9 ... "spd_Flag":2 ...
```

## 6. 切换场景

在当前场景脚本终端按`Ctrl+C`。看到：

```text
V0.6.12.8.2.2.81 test targets removed.
```

然后直接启动另一个`scenario_*.py`；不要停止CARLA和main.py。

## 7. RSM与预警事件的协议边界

main.py持续将融合参与者转换成大椽格式RSM，并发布到：

```text
command/dachuan/DC887-002047/req/{UUID}/rsm
```

- VRUCW和AVW的行人/车辆可以通过RSM发送给RSU，再由RSU通过PC5广播。
- 道路障碍物不是机动车、非机动车或行人，HLW语义不能只靠RSM表达。
- 道路限速属于道路标志，SLW限速值也不能放进RSM参与者字段。

当前按要求默认持续发送RSM，本地四种场景事件都会产生。如果以后要求OBU也直接收到
HLW和SLW语义，需要启用RSI并确认大椽映射，不能把`event2hmi`原样伪装成RSM。

## 8. 完整链路判定

1. main.py出现`[RSU MQTT TX] type=RSM`。
2. main.py收到`command///res/{UUID}/200`且`return_code=200`。
3. OBU的`rsus`或`local`中出现RSU广播的参与者数据。
4. VRUCW/AVW再观察OBU的`event2hmi`是否触发。

只看到第1步代表MEC调用了MQTT发送，不代表RSU解析或PC5链路已经成功。
