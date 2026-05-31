# ZeroErr-Motor-IGH-EtherCAT-Master-CST-DC-Mode

基于 IGH EtherCAT Master 的零差云控关节模组 CST（Cyclic Synchronous Torque，循环同步力矩）模式开发，目标是研究并实现 DC（Distributed Clocks，分布式时钟）同步控制。

## 项目概述

本项目在官方 CSV 示例代码的基础上，经过了"分析 → CSP 原型 → 模块化重构"的演进路径，最终实现了 **CST 模式**的通用 EtherCAT 主站框架。当前版本采用 **PD 控制器**（位置误差比例 + 速度阻尼）实现基于力矩控制的零力保持/位置锁定功能。

| 版本 | 说明 |
|------|------|
| V1.0.0 | 初始版本，基础框架搭建 |
| V1.0.x | 调试版本，CST 模式切换与 DC 同步验证 |
| **V1.0.1** | **当前版本**，PD 控制器参数需根据实际硬件调整 |

## 硬件环境

| 组件                     | 数量 |
| ------------------------ | ---- |
| 零差云控关节模组         | x1   |
| 零差云控 800W 电源       | x1   |
| Ubuntu 系统主机          | x1   |
| RJ45 网线-关节模组转接线 | x1   |

## 软件环境

- **操作系统**: Ubuntu（Linux，推荐 RT-PREEMPT 内核）
- **EtherCAT 主站**: [IGH EtherCAT Master](https://gitlab.com/etherlab.org/ethercat)（stable-1.5 / EtherLab）
- **构建工具**: CMake（>= 3.5）
- **编译器**: GCC / G++（>= 5.0）

## 项目结构

```
.
├── README.md
├── LICENSE                             # MIT License
└── src/
    └── CST/                            # CST 模式实现（通用 EtherCAT 主站模板）
        ├── CMakeLists.txt              # CMake 构建配置
        ├── README.md                   # 模块详细文档
        ├── include/                    # 头文件
        │   ├── csp_config.h            # [通用] 编译常量、宏、全局声明
        │   ├── csp_util.h              # [通用] 时间运算 & 系统工具声明
        │   ├── ecat_config.h           # [通用] 从站硬件配置声明 (PDO/同步/域)
        │   ├── ecat_master.h           # [通用] 主站操作声明 (初始化/DC/收发)
        │   └── cst_control.h           # [应用] CST 模式驱动控制声明
        ├── src/                        # 源文件
        │   ├── main.cpp                # 主入口 —— 组装各模块
        │   ├── csp_util.cpp            # [通用] 时间/CPU/内存/信号实现
        │   ├── ecat_config.cpp         # [通用] PDO 映射 & 域注册数据定义
        │   ├── ecat_master.cpp         # [通用] 主站初始化/DC 配置/收发封装
        │   └── cst_control.cpp         # [应用] CST 驱动状态机 & PD 控制逻辑
        └── build/                      # 构建输出目录
```

### 模块架构

```
┌──────────────────────────────────────────────────┐
│                    main.cpp                       │
│           (组装入口, 编译配置 flag)                │
├────────┬────────┬────────┬───────────┬───────────┤
│csp_util│csp_    │ecat_   │ecat_      │cst_       │
│        │config  │config  │master     │control    │
│ 时间   │ 常量   │PDO映射 │ 主站操作   │ 驱动控制   │
│ CPU    │ 宏     │同步器  │ DC配置    │ 状态机    │
│ 内存   │ 全局   │域注册  │ 收发循环   │ PD控制器  │
├────────┴────────┴────────┴───────────┴───────────┤
│              IGH EtherCAT Master (libethercat)     │
│              Linux + RT Kernel                     │
└──────────────────────────────────────────────────┘

  通用层 (任何 EtherCAT 项目复用)     应用层 (可替换)
```

### 模块职责

| 模块 | 类型 | 职责 | 更换时机 |
|------|------|------|----------|
| `csp_config.h` | 通用 | 周期常量、时间宏、硬件参数、全局声明 | 一般不改 |
| `csp_util` | 通用 | 时间运算、CPU 亲和性、实时调度、内存锁定 | 一般不改 |
| `ecat_config` | 通用 | 从站身份、PDO 条目、同步管理器、域注册表 | 换从站硬件时 |
| `ecat_master` | 通用 | 主站请求、PDO 配置、DC 同步、激活、循环收发 | 一般不改 |
| `cst_control` | **应用** | CiA 402 状态机、CST 力矩控制、PD 控制器 | 换控制模式时 |
| `main.cpp` | 入口 | 组装调用、编译 flag、调试开关 | 换应用场景时 |

## CST 控制原理

### PD 控制器

当前实现了一个简单的 **PD（比例-微分）控制器**，以位置误差为输入、输出目标力矩，实现位置保持：

```
error = holdPos - actPos          // 位置误差 (编码器单位)
Kp = 0.1                          // 比例增益
Kd = 0.001                        // 微分增益 (速度阻尼)

targetTorque = Kp × error ± 30 - Kd × actVel   // ±30 为死区补偿偏置
```

- **P 项**：位置误差越大，输出的力矩命令越大（弹性回复力）
- **D 项**：通过实际速度提供阻尼，防止振荡
- **死区补偿**：±30 的偏置用于克服静摩擦力

> ⚠️ **注意**：`Kp`、`Kd` 和死区偏置值定义在 [`cst_control.h`](src/CST/include/cst_control.h) 中，需根据实际硬件负载和关节特性进行调参。

### PDO 映射

CST 模式需要传输力矩相关对象字典条目：

| 方向 | 对象字典 | 含义 | 位宽 |
|------|----------|------|------|
| RxPDO (主→从) | `0x6040` | Controlword | 16 bit |
| RxPDO (主→从) | `0x607a` | Target Position | 32 bit |
| RxPDO (主→从) | `0x60FF` | Target Velocity | 32 bit |
| **RxPDO (主→从)** | **`0x6071`** | **Target Torque** | **16 bit** |
| TxPDO (从→主) | `0x6041` | Statusword | 16 bit |
| TxPDO (从→主) | `0x6064` | Position Actual | 32 bit |
| TxPDO (从→主) | `0x606c` | Velocity Actual | 32 bit |
| **TxPDO (从→主)** | **`0x6077`** | **Torque Actual** | **16 bit** |

## 关键技术概念

### EtherCAT 通信模型

```
对象字典 (Object Dictionary)
    ↓
PDO Mapping（决定传输哪些对象字典条目）
    ↓
PDO（过程数据对象，周期性实时数据交换）
    ↓
Domain（IGH 中的共享内存缓冲区，映射 PDO 数据）
    ↓
你的应用程序（通过指针直接读写 Domain 内存）
```

- **SDO（Service Data Object）**: 非周期性通信，用于配置参数（如模式切换、报警复位）
- **PDO（Process Data Object）**: 周期性实时通信，用于传输控制指令和反馈数据
- **RxPDO / TxPDO**: 以从站视角定义，RxPDO 为主站→从站，TxPDO 为从站→主站

### DC 分布式时钟

- DC 时钟是一个 64 位纳秒计数器，并非日历时间
- **Sync0**: 主同步脉冲信号，由硬件触发，用于在精确时刻同步所有从站执行指令
- 同步策略有两种：
  - **SYNC_REF_TO_MASTER**: PC 主站时钟为参考（本项目采用）
  - **SYNC_MASTER_TO_REF**: 从站 0 时钟为参考

### CiA 402 状态机

| 状态字   | 含义             | 对应控制字            |
| -------- | ---------------- | --------------------- |
| `0x0008` | 故障状态         | `0x0080` 故障复位     |
| `0x0040` | 伺服无故障       | `0x0006` 关闭         |
| `0x0021` | 伺服准备好       | `0x0007` 打开伺服使能 |
| `0x0023` | 等待打开伺服使能 | `0x000F` 伺服运行     |
| `0x0027` | 伺服运行         | `0x000F` 保持运行     |

```
                  ┌──────────────────────────┐
                  │      STATE_FAULT          │ ← 故障
                  │        (0x0008)           │
                  └──────────┬───────────────┘
                             │ CONTROL_WORD_FAULT_RESET (0x80)
                             ▼
                  ┌──────────────────────────┐
                  │  STATE_SWITCH_ON_DISABLED │ ← 初始状态
                  │        (0x0040)           │
                  └──────────┬───────────────┘
                             │ CONTROL_WORD_SHUTDOWN (0x06)
                             ▼
                  ┌──────────────────────────┐
                  │  STATE_READY_TO_SWITCH_ON │
                  │        (0x0021)           │
                  └──────────┬───────────────┘
                             │ CONTROL_WORD_SWITCH_ON (0x07)
                             ▼
                  ┌──────────────────────────┐
                  │    STATE_SWITCHED_ON      │
                  │        (0x0023)           │
                  └──────────┬───────────────┘
                             │ CONTROL_WORD_ENABLE_OPERATION (0x0F)
                             ▼
                  ┌──────────────────────────┐
                  │  STATE_OPERATION_ENABLED  │ ← CST 力矩控制
                  │        (0x0027)           │
                  └──────────────────────────┘
```

## 支持的操作模式

模块化架构支持多种 CiA 402 控制模式，只需替换 `cst_control` 模块即可切换：

| 模式 | 常量 | 说明 |
|------|------|------|
| PP | `0x01` | Profile Position (配置文件位置) |
| PV | `0x03` | Profile Velocity (配置文件速度) |
| PT | `0x04` | Profile Torque (配置文件力矩) |
| CSP | `0x08` | Cyclic Sync Position (循环同步位置) |
| CSV | `0x09` | Cyclic Sync Velocity (循环同步速度) |
| **CST** | **`0x0A`** | **Cyclic Sync Torque (循环同步力矩)** ← 当前实现 |

## 构建与运行

### 1. 安装 IGH EtherCAT Master

```bash
# 以 Ubuntu 为例，使用 EtherLab 发行版
sudo apt install ethercat-master libethercat-dev

# 或从源码编译
# git clone https://gitlab.com/etherlab.org/ethercat.git
# cd ethercat && git checkout stable-1.5
# ./bootstrap && ./configure && make && sudo make install
```

### 2. 启动 EtherCAT 服务

```bash
sudo /etc/init.d/ethercat start
```

### 3. 编译项目

```bash
cd src/CST/build
cmake ..
make -j$(nproc)
```

> **注意**：如果 EtherLab 安装在非默认路径（如 `/usr/local/etherlab`），请修改 [`CMakeLists.txt`](src/CST/CMakeLists.txt) 中的 `ETHERLAB_DIR` 变量。

### 4. 运行

```bash
sudo ./CSP_PROJECT
```

按 `Ctrl+C` 退出（程序会先释放主站资源再终止）。

### 5. 调试开关

在 [`main.cpp`](src/CST/src/main.cpp) 中取消注释以下宏以启用调试功能：

| 宏 | 功能 |
|----|------|
| `MEASURE_PERF` | 打印 DC 时钟偏差 |
| `MEASURE_TIMING` | 打印循环执行时间 |
| `SET_CPU_AFFINITY` | 绑定进程到指定 CPU 核心 |

## 控制流程

```
main()
 ├─ set_cpu_affinity()        ← 绑定 CPU 核心 (可选)
 ├─ set_realtime_priority()   ← SCHED_FIFO 最高优先级
 ├─ lock_memory()             ← 锁定内存防 swap
 ├─ stack_prefault()          ← 预分配栈空间
 ├─ ecat_master_request()     ← 请求主站实例
 ├─ initDrive()               ← SDO: 设置 CST 模式 (0x0A) + 复位报警
 ├─ ecat_slave_config()       ← 创建从站配置
 ├─ ecat_config_pdos()        ← 配置 PDO 映射 (含力矩条目)
 ├─ ecat_create_domain()      ← 创建过程数据域
 ├─ ecat_reg_pdo_entries()    ← 注册 PDO 条目偏移
 ├─ ecat_config_dc()          ← 配置分布式时钟
 ├─ ecat_activate()           ← 激活主站
 │
 ├─ [OP 等待循环]              ← 等待所有从站进入 Operational
 │   ├─ ecat_cycle_receive()
 │   ├─ 检测 slaveState.operational
 │   ├─ ecat_sync_dc()        ← DC 时钟同步
 │   └─ ecrt_master_send()
 │
 └─ [主控制循环]               ← 1ms 周期实时控制
     ├─ clock_nanosleep()     ← 精确周期睡眠
     ├─ ecat_cycle_receive()
     ├─ ecat_cycle_process()
     ├─ 读取 PDO (位置/速度/力矩/状态字)
     ├─ driveStateMachine()   ← CiA 402 状态机 + PD 控制器
     ├─ 写入 PDO (控制字/目标力矩)
     ├─ ecat_cycle_queue_and_send()
     └─ ecat_sync_dc()        ← DC 时钟同步
```

## 自定义指南

### 切换控制模式 (CST → CSP / CSV 等)

1. 创建对应的 `include/xxx_control.h` 和 `src/xxx_control.cpp`
2. 实现 `initDrive()` 和 `driveStateMachine()` 接口
3. 在 `main.cpp` 中将 `#include "cst_control.h"` 替换为新模块
4. 将 `initDrive(master, 0, CST)` 中的模式常量改为新模式

### 适配不同从站硬件

修改 [`ecat_config.cpp`](src/CST/src/ecat_config.cpp)：

```c
// 1. 修改从站身份
uint32_t vendor_id   = 0x YOUR_VID;
uint32_t product_code = 0x YOUR_PID;

// 2. 修改 PDO 条目 (按驱动器手册)
ec_pdo_entry_info_t slave_0_pdo_entries[] = { ... };

// 3. 修改 PDO 映射索引
ec_pdo_info_t slave_0_pdos[] = { ... };
```

### 添加新从站

1. 在 `ecat_config.cpp` 中添加新从站的 PDO 条目、PDO、同步管理器数组
2. 在 `domain1_regs[]` 中注册新从站的 PDO 偏移
3. 在 `main.cpp` 中创建对应的 `ec_slave_config_t*` 和状态检测

## 参考资源

- [IGH EtherCAT Master 官方文档](https://etherlab.org/en/ethercat/)
- [EtherLab 安装指南](https://etherlab.org/en/ethercat/software.php)
- [CiA 402 驱动规范](https://www.can-cia.org/can-knowledge/) — 驱动器状态机与控制字定义
- [CMake 入门教程](https://subingwen.cn/cmake/CMake-primer/)
- 零差云控关节模组官方手册（对象字典、PDO 映射定义）

---

**本项目持续更新中。** V1.0.1 版本 PD 控制器参数为默认值，实际使用时需根据关节负载特性自行调参。
