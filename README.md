# 仓库环境监测系统 v2.0 稳定版

基于 RT-Thread 和 STM32F407 的智能仓库环境监测系统

## 项目简介

本项目实现了一个基于嵌入式系统的仓库环境监测解决方案，集成了温湿度监测、光照检测、货架倾斜监测、舵机控制、LCD 显示和云端上传等功能。使用 RT-Thread 操作系统管理多任务调度。

## 硬件平台

- **主控芯片**: STM32F407ZGT6
- **无线通信**: RW007 (SPI WiFi 模块)
- **屏幕**: ST7789 240x240 LCD
- **传感器**:
  - AHT10 - 温湿度传感器 (I2C3)
  - AP3216C - 光照 + 接近传感器 (I2C2)
  - ICM20608 - 六轴加速度计+陀螺仪 (I2C2)
- **执行器**:
  - 舵机  - 货架角度控制 (PWM2 CH4)
  - 蜂鸣器 - 声音报警
  - LED   - 灯光控制

## 功能特性

### OS多线程调度 (RT-Thread)
| 优先级 | 线程 | 功能 |
|--------|------|------|
| 8 | logic | 逻辑处理 — 温度/倾斜告警→蜂鸣器+舵机，最高响应速度 |
| 10 | mqtt0 | PAHO MQTT 内部线程 |
| 15 | onenet | OneNET 数据上传 + 下行命令消费 |
| 18 | web | HTTP Web Server |
| 20 | LVGL | LCD 显示渲染 |
| 23 | http* | HTTP异步处理 |
| 25 | sensor | 传感器采集 (5秒周期) |

### OS同步原语使用
- **互斥量 (Mutex)**: `data_lock` 保护全局状态 `g_app_state`，防止多线程数据竞争
- **信号量 (Semaphore)**: `net_ready_sem` 同步WiFi就绪状态，HTTP/OneNET线程阻塞等待
- **事件集 (Event)**: `evt_alarm` 实现生产者-消费者模式（传感器→逻辑线程）

### 环境监测
- 实时监测仓库温度、湿度、光照、接近数据
- 货架 X/Y 轴倾斜角度检测 (ICM20608加速度计)
- 倾斜角度超过 15° 时触发报警

### 舵机控制
- 基于 PWM 信号控制舵机角度（0°-180°），告警时自动旋转至90°位置

### 显示界面
- LVGL 图形界面，BSP单"LVGL"线程处理所有UI操作，线程安全
- 中文字库支持，传感器数据图标化展示

### 云端通信 (OneNET)
- MQTT 协议上传环境数据 (30秒周期) + 告警数据 (状态变化触发)
- 物模型属性：temperature, humidity, light, proximity, tilt_angle, alarm_state, beep, temp_threshold, vibration, fan_status
- 下行命令控制：LED、BEEP、温度阈值设置

### 本地HTTP控制 (端口80)
- `GET /get_status` — 获取当前状态JSON
- `GET /api/set?type=threshold&val=40.0` — 设置温度阈值
- `GET /api/set?type=beep&val=1` — 控制蜂鸣器

### 报警功能
- 温度过高报警 (阈值可通过APP/按键/云端远程设置)
- 货架倾斜报警
- 蜂鸣器声音提示 + LCD 视觉报警

## 软件架构

```
applications/
├── main.c              # 主程序入口 — OS对象创建 + 线程启动
├── app_logic.c/h       # 业务逻辑中心 — 互斥量临界区保护
├── app_sensor.c/h      # 传感器采集模块 — 低优先级线程 + 事件集告警
├── app_net.c/h         # 网络通信模块 — 信号量同步 + ISR安全命令处理
├── lv_port_disp.c      # LVGL 显示驱动适配
├── my_font_cn_16.c     # 中文字库
├── Alarm.c             # 报警图标
├── Environmental.c     # 温度图标
├── Megaphone.c         # 蜂鸣器图标
├── connected.c         # WiFi 连接图标
├── light.c             # 光照图标
├── onenet_upload.c     # 上传图标
├── tilt.c              # 倾斜图标
└── waterprof.c         # 湿度图标
```

## 报警阈值
- 温度报警: 默认 40°C (可通过APP/按键/远程调整，范围20-50°C)
- 倾斜报警: X 或 Y 轴 > 15°

## 编译与烧录

使用 RT-Thread Studio IDE 开发：

1. 用 RT-Thread Studio 打开本项目
2. 项目右键 → Incremental Build 编译
3. 下载到开发板

## 配套Android APP

路径: [apk/onenet2.1.5-app-debug.apk](apk/onenet2.1.5-app-debug.apk)

功能:
- 实时查看温度、湿度数据
- 远程设置温度阈值
- 远程控制蜂鸣器、LED
- OneNET 物模型属性查看

## 目录结构

```
onenet/
├── applications/          # 应用代码
├── apk/                  # 配套Android APP
├── packages/             # RT-Thread 软件包
│   ├── aht10-latest/     # AHT10 驱动
│   ├── ap3216c-latest/   # AP3216C 驱动
│   ├── icm20608-latest/  # ICM20608 驱动
│   ├── onenet-latest/    # OneNET MQTT 接入
│   └── pahomqtt-latest/  # PAHO MQTT客户端
├── rtconfig.h            # RT-Thread 配置
└── README.md             # 项目说明文档

## 截图预览

LCD 显示界面包含以下区域：
- **顶部**: WiFi状态图标、OneNET上传图标、中文标题"仓库环境监测"
- **中部**: 温度、湿度、光照、倾斜角度数据展示
- **底部**: 阈值显示、报警状态、蜂鸣器图标

## 作者

仓库环境监测系统开发

## 致谢

- RT-Thread 操作系统 & 社区
- OneNET 物联网平台
- LVGL 图形库
- PAHO MQTT C/C++ client library
