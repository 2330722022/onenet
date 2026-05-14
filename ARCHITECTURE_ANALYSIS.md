# 《智能仓库环境感知与调控系统》软件架构与 IPC 机制深度分析

> **答辩准备材料**
> 平台：STM32F407 + RT-Thread 实时操作系统
> 传感器：AHT10(温湿度) / AP3216C(光强接近) / ICM20608(6轴IMU)
> 通信：RW007 WiFi + OneNET MQTT + 自建 HTTP Server

---

## 一、整体软件架构与线程划分

### 1.1 线程一览表

系统共创建了 **6 类关键线程**，按优先级从高到低排列如下：

| 线程名称 | 优先级 | 栈大小 | 创建位置 | 核心职责 |
|:---|:---:|:---:|:---|:---|
| **logic** | **8** | 2048 | `app_logic.c` | BEEP/舵机硬件执行、告警状态切换 |
| paho_mqtt | 10 | — | PAHO 内部库 | MQTT 协议栈保活、心跳维持 |
| **onenet** | **15** | 3072 | `app_net.c` | OneNET 数据上传、下行命令消费 |
| **web** | **18** | 2048 | `app_net.c` | HTTP Server 主 acceptor 线程 |
| LVGL | 20 | 8192 | BSP 内部 | `lv_timer_handler()` 驱动 GUI 刷新 |
| **http** (worker) | **23** | 2048 | `app_net.c` | 每个 HTTP 请求的独立处理线程 |
| **sensor** | **25** | 3072 | `app_sensor.c` | AHT10/AP3216C/ICM20608 周期采集 |

### 1.2 优先级设计思想

这个优先级分配表是整个系统**最核心的架构决策**，体现了 RTOS 抢占式调度的精髓：

**为什么逻辑线程优先级最高（prio=8）？**

逻辑线程负责直接操作硬件执行器（BEEP 蜂鸣器、舵机），这是安全关键行为。当 ICM20608 检测到货架倾斜超过 15 度时，必须**毫秒级**内切断危险。如果优先级不够高，可能被其他线程阻塞导致响应延迟。

```c
// 传感器线程检测到倾斜 → 通过事件集异步通知逻辑线程
// 文件: app_sensor.c
rt_event_send(&evt_alarm, EVENT_TILT_ALARM);
```

```c
// 逻辑线程以最高优先级等待并立即响应
// 文件: app_logic.c
rt_event_recv(&evt_alarm,
              EVENT_TEMP_ALARM | EVENT_TILT_ALARM |
              EVENT_VIBRATION_ALARM | EVENT_ALARM_CLEAR, ...)
// 收到事件后立即操作硬件：BEEP引脚拉高 + 舵机转到90度
rt_pin_write(BEEP_PIN, PIN_HIGH);
servo_set_angle(90);
```

**为什么传感器采集线程优先级最低（prio=25）？**

传感器采集周期为 5 秒（`rt_thread_mdelay(5000)`），这是慢速任务。LVGL 显示需要 20ms 级刷新，HTTP 需要及时响应 Android 轮询。将传感器设为最低优先级，意味着**只要任何高优先级线程就绪，传感器立即让出 CPU**——这就是"优先级反转防护"的正面设计。

### 1.3 线程拓扑图

```
main() 启动顺序
  │
  ├─ app_logic_init()      → 创建 data_lock + evt_alarm → 启动 logic(prio=8)
  ├─ app_sensor_init()     → 创建 data_mutex → 启动 sensor(prio=25)
  └─ app_net_init()        → 创建 net_ready_sem → 启动 web(prio=18)
                              ↓ (WiFi 连接后)
                           app_net_onenet_start() → 启动 onenet(prio=15)
```

---

## 二、核心多线程通信机制 (IPC Mechanisms)

系统中使用了三种 RT-Thread IPC 内核对象 + 一种 ISR 安全标志位模式，下面逐一分析。

### 2.1 互斥量 (Mutex) — 防止数据竞争与 Hard Fault

系统中创建了 **两个互斥量**，分工明确：

#### 互斥量 1：`data_mutex` — 保护传感器共享数据

- **创建位置**：`app_sensor.c` 的 `app_sensor_init()` 中
- **保护对象**：`shared_data` 结构体 + `status_json` / `data_json` 全局 JSON 缓冲区
- **竞争场景**：传感器线程每 5 秒写入数据，同时 HTTP 线程可能正在读取并 `send()` 给 Android App，LVGL 线程也在读取用于 UI 刷新

```c
// 传感器线程 — 写入临界区 (app_sensor.c)
rt_mutex_take(data_mutex, RT_WAITING_FOREVER);
shared_data = data;       // 结构体整体赋值（原子化）
// 构建 JSON 供 HTTP 零拷贝引用
rt_snprintf(status_json, sizeof(status_json), "{...}");
rt_mutex_release(data_mutex);
```

```c
// LVGL UI 线程 — 读取临界区（非阻塞尝试）(main.c)
if (rt_mutex_take(data_mutex, 1) == RT_EOK) {
    local_data = shared_data;
    rt_mutex_release(data_mutex);
} else {
    return;  // 数据正被写入，跳过本轮刷新
}
```

**工程痛点解决**：如果没有 `data_mutex`，传感器线程写到一半时被 HTTP 线程抢占，HTTP 线程读取到的 `shared_data.temperature` 可能是旧值、`shared_data.humidity` 可能是新值——这就是典型的**数据竞争 (Data Race)**。更严重的是，`status_json[256]` 是定长 char 数组，如果没有互斥，一个线程在写入 `snprintf` 过程中被另一个线程抢占并读取残缺 JSON，Android 端解析会直接崩溃。

#### 互斥量 2：`data_lock` — 保护全局业务状态

- **创建位置**：`app_logic.c` 的 `app_logic_init()` 中
- **保护对象**：`g_app_state` 结构体（含 `temp_threshold`、`beep_status`、`alarm_state`）
- **竞争场景**：Android HTTP、本地按键中断、OneNET 云端下行、逻辑线程都可能同时修改阈值

```c
// HTTP 线程 — Android 远程设置阈值 (app_net.c)
rt_mutex_take(data_lock, RT_WAITING_FOREVER);
g_app_state.temp_threshold = new_threshold;
rt_mutex_release(data_lock);
```

```c
// 按键事件处理（在传感器线程中轮询消费）(app_logic.c)
rt_mutex_take(data_lock, RT_WAITING_FOREVER);
g_app_state.temp_threshold += 0.5f;   // 与 HTTP 线程互斥
rt_mutex_release(data_lock);
```

**工程痛点解决**：假设用户同时按下本地按键（+0.5）和 Android App 发送设置阈值（35.0），无锁时可能产生：
- 线程 A 读取 threshold=30.0
- 线程 B 写入 threshold=35.0
- 线程 A 写入 threshold=30.5（期望值应为 35.5！）

这就是典型的**读-改-写非原子操作**导致的竞态。`data_lock` 确保这两个操作不会交织。

### 2.2 信号量 (Semaphore) — WiFi 就绪同步

- **创建位置**：`app_net.c` 的 `app_net_init()` 中
- **内核对象**：`net_ready_sem`（初始值 = 0）

**经典的生产者-消费者模型：**

```
生产者 (WiFi 连接成功)
  └─ rt_sem_release(net_ready_sem)  → 释放 2 次

消费者 1 (Web Server 线程, prio=18)
  └─ rt_sem_take(net_ready_sem, RT_WAITING_FOREVER)
     └─ 阻塞挂起，不消耗 CPU → 信号量可用 → 被唤醒 → 启动 HTTP listen

消费者 2 (OneNET 上传线程, prio=15)
  └─ rt_sem_take(net_ready_sem, RT_WAITING_FOREVER)
     └─ 阻塞挂起，不消耗 CPU → 信号量可用 → 被唤醒 → MQTT 初始化
```

```c
// 文件: app_net.c
rt_sem_release(net_ready_sem);  // 消费者1: HTTP Server
rt_sem_release(net_ready_sem);  // 消费者2: OneNET 上传
```

**工程痛点解决**：如果 WiFi 尚未获取 IP 地址，HTTP Server 和 OneNET 线程就启动 `socket()` 和 `onenet_mqtt_init()`，会立即失败。传统裸机方案需要在 `while(1)` 中轮询 WiFi 状态标志，浪费 CPU。RT-Thread 信号量让消费者线程在**阻塞挂起状态等待**——不参与调度、不消耗 CPU 周期，WiFi 就绪后内核自动唤醒，实现零开销等待。

### 2.3 事件集 (Event) — 传感器告警异步通知

- **创建位置**：`app_logic.c` 的 `app_logic_init()` 中
- **内核对象**：`evt_alarm`

**事件定义：**

```c
// 文件: app_logic.h
#define EVENT_TEMP_ALARM       (1 << 0)   // 温度超限
#define EVENT_TILT_ALARM       (1 << 1)   // 货架倾斜
#define EVENT_VIBRATION_ALARM  (1 << 2)   // 振动检测
#define EVENT_ALARM_CLEAR      (1 << 3)   // 告警解除
```

**生产者-消费者模式：**

```
生产者 (传感器线程, prio=25)
  ├─ 温度过高 → rt_event_send(&evt_alarm, EVENT_TEMP_ALARM)
  ├─ 倾斜超限 → rt_event_send(&evt_alarm, EVENT_TILT_ALARM)
  └─ 振动异常 → rt_event_send(&evt_alarm, EVENT_VIBRATION_ALARM)

消费者 (逻辑线程, prio=8 — 最高优先级！)
  └─ rt_event_recv(&evt_alarm, ..., RT_WAITING_FOREVER)
     └─ 阻塞挂起 → 事件到达 → 内核立即唤醒 → 抢占执行 BEEP/舵机
```

```c
// 文件: app_logic.c — 逻辑线程主循环
if (rt_event_recv(&evt_alarm,
                  EVENT_TEMP_ALARM | EVENT_TILT_ALARM |
                  EVENT_VIBRATION_ALARM | EVENT_ALARM_CLEAR,
                  RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                  RT_WAITING_FOREVER, &events) == RT_EOK)
{
    if (events & EVENT_TEMP_ALARM) {
        rt_pin_write(BEEP_PIN, PIN_HIGH);  // 拉高GPIO，蜂鸣器响
        servo_set_angle(90);               // 舵机转到90度
    }
}
```

**为什么不用消息队列？** 消息队列适用于需要传递**具体数据内容**的场景（如传输一个温度数值）。而本系统的告警场景只需要传递**哪类事件发生**（位掩码即可表达），事件集比消息队列更轻量：无需内存拷贝、无需动态分配消息块、多个事件可合并为单一 `rt_event_send` 调用。对于"温度过高 → 响蜂鸣器"这种纯控制信号，事件集是更优的选择。

### 2.4 补充：ISR 安全的 volatile 标志位模式 — OneNET 下行回调

这是一个非常重要的架构细节。OneNET 的 MQTT 命令回调 `onenet_cmd_rsp_cb` 可能运行在 **paho_mqtt 内部线程甚至中断上下文**，此时禁止调用任何阻塞 API（如 `rt_mutex_take`）。

```c
// 【回调函数】ISR 安全 — 仅写 volatile 标志位 (app_net.c)
static void onenet_cmd_rsp_cb(uint8_t *recv_data, ...) {
    // ...解析JSON...
    net_beep_on = 1;            // volatile 写，单字节原子操作
    net_beep_pending = 1;       // ISR 安全：ARM Cortex-M 单字节访问是原子的
}

// 【上传线程】主循环中异步消费标志位 (app_net.c)
if (net_beep_pending) {
    net_beep_pending = 0;
    rt_mutex_take(data_lock, RT_WAITING_FOREVER);  // 安全：线程上下文可阻塞
    g_app_state.beep_status = net_beep_on;
    rt_pin_write(BEEP_PIN, net_beep_on ? PIN_HIGH : PIN_LOW);
    rt_mutex_release(data_lock);
}
```

### 2.5 IPC 机制总结对比

| IPC 对象 | 工程痛点 | 解决方式 |
|:---|:---|:---|
| **`data_mutex`** (互斥量) | 传感器数据被多线程同时读写导致数据竞争 | 写者全量锁定，读者非阻塞尝试 |
| **`data_lock`** (互斥量) | 阈值/告警状态被 4 个来源同时修改 | 所有写操作序列化，读-改-写原子化 |
| **`net_ready_sem`** (信号量) | 网络未就绪时 HTTP/MQTT 启动即失败 | 消费者阻塞挂起，生产者释放后自动唤醒 |
| **`evt_alarm`** (事件集) | 传感器需要低延迟通知逻辑线程执行告警 | 事件驱动，接收线程阻塞等待不轮询 |
| **volatile 标志位** (ISR 安全) | MQTT 回调可能在中断上下文无法阻塞 | 回调只写标志位，线程安全消费 |

---

## 三、核心数据流与控制流追踪

### 3.1 下行控制流：Android App → 开启蜂鸣器

**完整链路（7步）：**

```
[1] Android App
    └─ HTTP GET /api/set?type=beep&val=1
[2] web 线程 (prio=18)
    └─ accept() 接收连接
    └─ 创建 http worker 线程 (prio=23)
[3] http worker 线程 (prio=23)
    └─ recv() 读取 HTTP 请求
    └─ 解析 strstr(buffer, "type=beep") + "val=1"
    └─ 调用 beep_set(1)
[4] beep_set() 函数
    └─ rt_mutex_take(data_lock, RT_WAITING_FOREVER)  ← 进入临界区
    └─ g_app_state.beep_status = 1;
    └─ rt_pin_write(BEEP_PIN, PIN_HIGH);             ← 直接操作 GPIO！
    └─ rt_mutex_release(data_lock)
[5] GPIO 引脚 (PB0)
    └─ 输出高电平 → 三极管导通 → 蜂鸣器发声
[6] 同时：HTTP 响应
    └─ send(client_fd, "{\"status\":\"ok\",...}")
[7] 同时：LVGL 线程 (prio=20)
    └─ lv_ui_refresh_task 读取 data_lock (2 tick 超时)
    └─ beep==1 → 显示 BEEP 图标
```

```c
// 文件: app_net.c — HTTP 请求处理关键逻辑
else if (strcmp(type_buf, "beep") == 0) {
    int new_beep = atoi(val_buf);
    if (new_beep == 0 || new_beep == 1) {
        beep_set((uint8_t)new_beep);  // ← 内部持有 data_lock
        // ...返回HTTP响应...
    }
}
```

### 3.2 上行数据流：传感器超温 → 瞬间触发报警

**完整链路（5步）：**

```
[1] sensor 线程 (prio=25) — 每5秒采集
    ├─ aht10_read_temperature()  → temperature=42.5°C
    ├─ icm20608_get_accel()      → tilt_angle
    └─ ap3216c_read_ambient_light() → light
[2] logic_handle(42.5)  — 阈值判定
    ├─ rt_mutex_take(data_lock, RT_WAITING_NO)  ← 非阻塞尝试
    ├─ temperature(42.5) > threshold(40.0)  → 超温！
    └─ rt_event_send(&evt_alarm, EVENT_TEMP_ALARM)  ← 发送事件
[3] 内核调度器
    └─ 检测到 logic 线程 (prio=8) 事件就绪
    └─ 立即抢占当前 sensor 线程 (prio=25)
    └─ 切换上下文到 logic 线程
[4] logic 线程 (prio=8) — 几乎零延迟响应
    ├─ rt_event_recv() 返回 EVENT_TEMP_ALARM
    ├─ rt_mutex_take(data_lock, RT_WAITING_FOREVER)
    ├─ rt_pin_write(BEEP_PIN, PIN_HIGH)       ← BEEP 响
    └─ servo_set_angle(90)                    ← 舵机转
[5] 并行的 onenet 线程 (prio=15)
    └─ 检测 alarm_state 变化
    └─ upload_alarm_data() → MQTT publish 到 OneNET 云端
    └─ Android App 收到告警推送
```

```c
// 文件: app_logic.c — 阈值判定与事件发送
void logic_handle(float temperature) {
    // ...原子读取阈值...
    if (temperature > threshold) {
        should_alarm = 1;
        g_app_state.alarm_state = 1;
    }
    rt_mutex_release(data_lock);

    if (should_alarm) {
        rt_event_send(&evt_alarm, EVENT_TEMP_ALARM);  // ← 唤醒高优先级线程
    }
}
```

**核心看点：从传感器采集到 GPIO 翻转，延迟仅取决于线程切换时间（通常 < 50μs），这是裸机大循环无法做到的。**

---

## 四、RT-Thread 带来的决定性提升

> 引入 RT-Thread 实时操作系统相比于传统单片机"超级大循环（`while(1)`）"带来了三点决定性的稳定性提升：

### 4.1 抢占式调度消除时序耦合

裸机大循环中，传感器的 5 秒采样周期会阻塞蜂鸣器的紧急响应——如果在 `AHT10_Read()` 的 I2C 等待期间发生了倾斜告警，蜂鸣器必须等传感器读取完成才能响应，延迟在毫秒到秒级不可控。RT-Thread 的多线程抢占调度确保 **`logic` 线程（优先级 8）可以随时打断 `sensor` 线程（优先级 25）**，告警响应延迟确定性降到微秒级。

```
裸机模式: [I2C读取中...|I2C读取中...|...蜂鸣器响应...]  ← 延迟不可控
RTOS模式: [sensor采集]←抢占→[logic响应(50μs)]→[sensor继续]  ← 延迟确定
```

### 4.2 IPC 机制解决数据竞争

裸机中，中断服务程序和主循环共享全局变量时，要么关中断（增加抖动），要么忍受数据不一致。本系统通过两个互斥量（`data_mutex` 保护传感器数据、`data_lock` 保护业务状态）、一个信号量（`net_ready_sem` 实现 WiFi 就绪同步）、一个事件集（`evt_alarm` 实现告警异步通知），将多线程间所有共享资源的访问都纳入了操作系统内核的管理之下——**任何数据竞争都会导致 `rt_mutex_take` 阻塞而非内存踩踏**，杜绝了 Hard Fault 的根源。

### 4.3 事件驱动模型实现零开销等待

逻辑线程通过 `rt_event_recv` 挂起等待告警事件，不消耗 CPU；OneNET 和 HTTP 线程通过 `rt_sem_take` 挂起等待网络就绪——这些在裸机中都必须用 `while(flag==0){...}` 轮询，不仅浪费 CPU 周期，还增加了功耗。RT-Thread 让 CPU 在空闲时执行 `idle` 任务并进入低功耗模式，这是智能物联网设备电池供电场景下的关键能力。

---

## 附录：关键文件索引

| 文件 | 职责 | 核心函数/对象 |
|:---|:---|:---|
| `main.c` | 系统入口、LVGL UI 初始化、模块启动序列 | `lv_ui_refresh_task()` |
| `app_logic.c` | 业务逻辑中心、BEEP/舵机控制、按键处理 | `data_lock`, `evt_alarm`, `logic_thread_entry()` |
| `app_logic.h` | 业务状态结构体、事件宏定义 | `struct app_state`, `EVENT_TEMP_ALARM` |
| `app_sensor.c` | 传感器采集、共享数据保护 | `data_mutex`, `shared_data`, `sensor_thread_entry()` |
| `app_sensor.h` | 传感器数据结构声明 | `struct sensor_data` |
| `app_net.c` | HTTP Server、OneNET 上传/下行、WiFi 管理 | `net_ready_sem`, `web_server_thread_entry()`, `onenet_cmd_rsp_cb()` |
| `app_net.h` | 网络模块接口声明 | `net_ready_sem`, `wifi_connected` |
| `rtconfig.h` | RT-Thread 内核配置 | `RT_USING_SEMAPHORE`, `RT_USING_MUTEX`, `RT_USING_EVENT` |
