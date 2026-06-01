/* ============================================================
 * main.cpp — EtherCAT 通用主站模板入口
 *
 * 核心配置 flag (在 include/csp_config.h 中定义):
 *   CONFIG_PDOS - 使能 PDO 配置
 *   DC          - 使能分布式时钟
 *
 * 调试 flag (按需取消注释):
 *   MEASURE_PERF    - 测量参考从站时钟时间戳差值
 *   MEASURE_TIMING  - 测量循环执行时间
 *   SET_CPU_AFFINITY - 绑定 CPU 核心
 *
 * 更换控制模式: 替换 cst_control 模块即可
 * ============================================================ */

/* --- 调试开关 (按需启用) --- */
/* #define MEASURE_PERF */
/* #define MEASURE_TIMING */
/* #define SET_CPU_AFFINITY */

#include "csp_config.h"
#include "csp_util.h"
#include "ecat_config.h"
#include "ecat_master.h"
#include "cst_control.h"

#include <stdio.h>
#include <signal.h>
#include <inttypes.h>

/* 全局主站指针 (各模块通过 extern 访问) */
ec_master_t *master;

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* ---- 1. 系统初始化 ---- */
#ifdef SET_CPU_AFFINITY
    set_cpu_affinity(4);
#endif
    set_realtime_priority();
    lock_memory();
    stack_prefault();
    signal(SIGINT, signal_handler);

    /* ---- 2. EtherCAT 主站初始化 ---- */
    master = ecat_master_request();
    if (!master)
        return -1;

    initDrive(master, 0, CST); //初始化关节模组

    ec_slave_config_t *sc = ecat_slave_config(master);
    if (!sc)
        return -1;

    if (ecat_config_pdos(sc))
        return -1;

    ec_domain_t *domain = ecat_create_domain(master);

    printf("Activating master...\n");
    if (ecat_reg_pdo_entries(domain))
        return -1;

    ecat_config_dc(sc);

#ifdef SYNC_REF_TO_MASTER
    {
        struct timespec masterInitTime;
        clock_gettime(CLOCK_MONOTONIC, &masterInitTime);
        ecrt_master_application_time(master, TIMESPEC2NS(masterInitTime));
    }
#endif

    if (ecat_activate(master))
        return -1;

    uint8_t *domain_pd = ecat_domain_data(domain);
    if (!domain_pd)
        return -1;

    struct timespec cycleTime = {0, PERIOD_NS};

    /* ---- 3. 等待所有从站进入 OP 状态 ---- */
    {
        struct timespec wakeupTime;
        clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

        ec_slave_config_state_t slaveState;

        while (1)
        {
            timespec_add(&wakeupTime, &wakeupTime, &cycleTime);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

            ecat_cycle_receive(master);
            ecrt_slave_config_state(sc, &slaveState);

            if (slaveState.operational)
            {
                printf("All slaves have reached OP state\n");
                break;
            }

            ecrt_domain_queue(domain);
            ecat_sync_dc(master);
            ecrt_master_send(master);
        }
    }

    /* ---- 4. 运动控制主循环 ---- */
    int32_t targetPos = 0;
    int32_t actTorque = 0, targetTorque = 0;
    struct timespec wakeupTime, sleepTime;

#ifdef MEASURE_PERF
    uint32_t t_cur, t_prev;
#endif

    sleepTime = cycleTime;
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

    while (1)
    {
#ifdef MEASURE_TIMING
        struct timespec endTime, execTime;
        clock_gettime(CLOCK_MONOTONIC, &endTime);
        timespec_sub(&execTime, &endTime, &wakeupTime);
        printf("Execution time: %lu ns\n", execTime.tv_nsec);
#endif

        /* 精确周期睡眠 */
        timespec_add(&wakeupTime, &wakeupTime, &sleepTime);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

        /* 接收帧 & 处理域数据 */
        ecat_cycle_receive(master);
        ecat_cycle_process(domain);

#ifdef MEASURE_PERF
        ecrt_master_reference_clock_time(master, &t_cur);
#endif

        /* --- 读取过程数据 --- */
        actTorque = EC_READ_S16(domain_pd + offset_actual_torque);
        targetTorque = EC_READ_S16(domain_pd + offset_target_torque);
        uint16_t statusWord = EC_READ_U16(domain_pd + offset_statusword);

        /* --- CST 驱动状态机 --- */
        uint16_t state = driveStateMachine(statusWord, domain_pd,
                                           offset_controlword,
                                           offset_target_torque,
                                           &targetTorque);

        printf("targetTorque=%d actTorque=%d state=0x%04x\n",
               targetTorque, actTorque, state);

        /* 队列 & 发送 */
        ecat_cycle_queue_and_send(master, domain);

        /* DC 时钟同步 */
        ecat_sync_dc(master);

#ifdef MEASURE_PERF
        printf("\nTimestamp diff: %" PRIu32 " ns\n\n", t_cur - t_prev);
        t_prev = t_cur;
#endif
    }

    return 0;
}
