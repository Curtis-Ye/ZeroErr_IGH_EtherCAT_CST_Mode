# EtherCAT 通用主站模板

基于 **IGH EtherCAT Master** (EtherLab) 的通用运动控制模板，采用模块化架构，支持 CSP/CSV/CST/PP/PV/PT 等多种 CiA 402 控制模式。

## 目录结构

```
.
├── CMakeLists.txt              # CMake 构建配置
├── README.md
├── include/
│   ├── csp_config.h            # [通用] 编译常量、宏、全局声明
│   ├── csp_util.h              # [通用] 时间运算 & 系统工具声明
│   ├── ecat_config.h           # [通用] 从站硬件配置声明 (PDO/同步/域)
│   ├── ecat_master.h           # [通用] 主站操作声明 (初始化/DC/收发)
│   └── csp_control.h           # [应用] CSP 模式驱动控制声明
└── src/
    ├── main.cpp                # 主入口 —— 组装各模块
    ├── csp_util.cpp            # [通用] 时间/CPU/内存/信号实现
    ├── ecat_config.cpp         # [通用] PDO 映射 & 域注册数据定义
    ├── ecat_master.cpp         # [通用] 主站初始化/DC 配置/收发封装
    └── csp_control.cpp         # [应用] 驱动状态机 & CSP 控制逻辑
```

## 模块架构

```
┌──────────────────────────────────────────────────┐
│                    main.cpp                       │
│           (组装入口, 编译配置 flag)                │
├────────┬────────┬────────┬───────────┬───────────┤
│csp_util│csp_    │ecat_   │ecat_      │csp_       │
│        │config  │config  │master     │control    │
│ 时间   │ 常量   │PDO映射 │ 主站操作   │ 驱动控制   │
│ CPU    │ 宏     │同步器  │ DC配置    │ 状态机    │
│ 内存   │ 全局   │域注册  │ 收发循环   │ CSP逻辑   │
├────────┴────────┴────────┴───────────┴───────────┤
│              IGH EtherCAT Master (libethercat)     │
│              Linux + RT Kernel                     │
└──────────────────────────────────────────────────┘

  通用层 (任何 EtherCAT 项目复用)     应用层 (可替换)
```

### 模块职责

| 模块 | 类型 | 行数 | 职责 | 更换时机 |
|------|------|------|------|----------|
| `csp_config.h` | 通用 | 52 | 周期常量、时间宏、硬件参数、全局声明 | 一般不改 |
| `csp_util` | 通用 | 130 | 时间运算、CPU 亲和性、实时调度、内存锁定 | 一般不改 |
| `ecat_config` | 通用 | 134 | 从站身份、PDO 条目、同步管理器、域注册表 | 换从站硬件时 |
| `ecat_master` | 通用 | 165 | 主站请求、PDO 配置、DC 同步、激活、循环收发 | 一般不改 |
| `csp_control` | **应用** | 139 | CiA 402 状态机、CSP 位置控制、SDO 写入 | 换控制模式时 |
| `main.cpp` | 入口 | 175 | 组装调用、编译 flag、调试开关 | 换应用场景时 |

## 依赖环境

| 组件 | 版本要求 | 说明 |
|------|----------|------|
| Linux (with RT-PREEMPT) | 4.x / 5.x | 实时内核 |
| IGH EtherCAT Master | 1.5+ | EtherLab 发行版 |
| CMake | ≥ 3.5 | 构建系统 |
| GCC / G++ | ≥ 5.0 | C++ 编译器 |

```bash
# 安装 EtherLab (以 Ubuntu 为例)
sudo apt install ethercat-master libethercat-dev
# 或从源码编译: https://gitlab.com/etherlab.org/ethercat
```

## 构建 & 运行

```bash
cd build
cmake ..
make -j$(nproc)

# 启动 EtherCAT 主站 (需要 root 权限)
sudo /etc/init.d/ethercat start

# 运行程序
sudo ./CSP_PROJECT
```

> **注意:** 如果 EtherLab 安装在非默认路径，修改 `CMakeLists.txt` 中的 `ETHERLAB_DIR` 变量。

## 配置说明

### 编译配置 (csp_config.h)

```c
#define CONFIG_PDOS    // 使能 PDO 配置 (默认开启)
#define DC             // 使能分布式时钟 (默认开启)
```

通过 `#ifndef` 保护，也可用 CMake 命令行覆盖：

```bash
cmake .. -DCONFIG_PDOS=ON
```

### 调试开关 (main.cpp)

```c
// 取消注释以启用调试功能:
#define MEASURE_PERF     // 打印 DC 时钟偏差 (需 DC 开启)
#define MEASURE_TIMING   // 打印循环执行时间
#define SET_CPU_AFFINITY // 绑定到指定 CPU 核心
```

## 自定义指南

### 更换控制模式 (CSP → CSV / CST / PP 等)

1. 创建 `include/csv_control.h` 和 `src/csv_control.cpp`
2. 实现 `initDrive()` 和 `driveStateMachine()` (参考 `csp_control.cpp`)
3. 在 `main.cpp` 中将 `#include "csp_control.h"` 替换为新模块
4. 将 `initDrive(master, 0, CSP)` 中的模式常量改为新模式

### 更换从站硬件 (电机 / 驱动器型号)

修改 `ecat_config.cpp`：

```c
// 1. 修改从站身份
uint32_t vendor_id   = 0x YOUR_VID;
uint32_t product_code = 0x YOUR_PID;

// 2. 修改 PDO 条目 (按驱动器手册)
ec_pdo_entry_info_t slave_0_pdo_entries[] = { ... };

// 3. 修改 PDO 映射索引 (0x1600=RxPDO, 0x1a00=TxPDO)
ec_pdo_info_t slave_0_pdos[] = { ... };
```

### 添加新从站

1. 在 `ecat_config.cpp` 中添加新从站的 PDO 条目、PDO、同步管理器数组
2. 在 `domain1_regs[]` 中注册新从站的 PDO 偏移
3. 在 `main.cpp` 中创建对应的 `ec_slave_config_t*` 和状态检测

## 控制流程

```
main()
 ├─ set_cpu_affinity()        ← 绑定 CPU 核心
 ├─ set_realtime_priority()   ← SCHED_FIFO 最高优先级
 ├─ lock_memory()             ← 锁定内存防 swap
 ├─ stack_prefault()          ← 预分配栈空间
 ├─ ecat_master_request()     ← 请求主站实例
 ├─ initDrive()               ← SDO: 设置 CSP 模式 + 复位报警
 ├─ ecat_slave_config()       ← 创建从站配置
 ├─ ecat_config_pdos()        ← 配置 PDO 映射
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
     ├─ 读取 PDO (位置/速度/状态字)
     ├─ driveStateMachine()   ← CiA 402 状态机
     ├─ 写入 PDO (控制字/目标位置)
     ├─ ecat_cycle_queue_and_send()
     └─ ecat_sync_dc()        ← DC 时钟同步
```

## CiA 402 驱动状态机

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
                  │  STATE_OPERATION_ENABLED  │ ← CSP 位置控制
                  │        (0x0027)           │
                  └──────────────────────────┘
```

## 支持的操作模式

| 模式 | 常量 | 说明 |
|------|------|------|
| PP | `0x01` | Profile Position (配置文件位置) |
| PV | `0x03` | Profile Velocity (配置文件速度) |
| PT | `0x04` | Profile Torque (配置文件力矩) |
| **CSP** | **`0x08`** | **Cyclic Sync Position (循环同步位置)** ← 当前实现 |
| CSV | `0x09` | Cyclic Sync Velocity (循环同步速度) |
| CST | `0x0A` | Cyclic Sync Torque (循环同步力矩) |

## 文件统计

```
include/             5 个头文件,  245 行
src/                 5 个源文件,  550 行
CMakeLists.txt                    32 行
──────────────────────────────────────
合计                            827 行
```

## 参考资料

- [IGH EtherCAT Master 官方文档](https://etherlab.org/en/ethercat/)
- [EtherLab 安装指南](https://etherlab.org/en/ethercat/software.php)
- [CiA 402 驱动规范](https://www.can-cia.org/can-knowledge/)- 驱动器状态机与控制字
- 原始项目: `ZeroErr/CSP-DC`
