---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 495757a73352da7aae1b306638d4f5fe_90efc76ed69611f1a216525400248c00
    ReservedCode1: S39P/cNcfCGIJ7GQv5M2CGIeQmgmYqinnVOJrTQ3u+SJNiQovknO42ccgu8SXu8+DBRa5wgL7tETicDGx+QIwRzFW9u9/MbK1dVZLb7UgzJXTrFxdECDJGj2mKhp/tWmR9cbXmWcfYkapGckU7DpT/raDGPpbrGO52LkMOvO5LGf125/XoKOvtXml1M=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 495757a73352da7aae1b306638d4f5fe_90efc76ed69611f1a216525400248c00
    ReservedCode2: S39P/cNcfCGIJ7GQv5M2CGIeQmgmYqinnVOJrTQ3u+SJNiQovknO42ccgu8SXu8+DBRa5wgL7tETicDGx+QIwRzFW9u9/MbK1dVZLb7UgzJXTrFxdECDJGj2mKhp/tWmR9cbXmWcfYkapGckU7DpT/raDGPpbrGO52LkMOvO5LGf125/XoKOvtXml1M=
---







# STM8S103F3P6 project5 — HC-SR04 超声波测距 + 2 位 7 段数码管显示 + UART 调试 + BEEP

本工程基于 **STM8S103F3P6（TSSOP20）**，采用 **SDCC** 编译，工程结构与编码风格与
`project4` 完全一致：面向对象封装（结构体 + 函数指针）、`main.c` 集中定义 ISR、
TIM1 1ms 时间片调度、数码管动态扫描、UART 调试输出。与 project4 的关键区别：

- **外设更换**：去掉 I2C（不再使用 PB4/PB5），新增 **HC-SR04 超声波测距**；
- **数码管更换**：四位共阴改为**两位共阳**（段码表取反为低电平点亮、位选高电平
  选中，位选只需 2 个）；
- **新增 BEEP**：复用 project1 的 `beep.c/beep.h`（硬件 BEEP 外设 PD4，LSI 时钟，
  1/2/4kHz），有效测距给出提示音（无效 / 无回波不响）。

- MCU: STM8S103F3P6（TSSOP20）
- 时钟: HSI 16MHz（无外部晶振）
- 编译器: SDCC 4.5.0（`sdcc -mstm8 --std-c99`）
- 烧录: stm8flash（ST-LINK/V2）

## 1. 功能概述

| 功能 | 说明 |
|---|---|
| HC-SR04 测距 | PB5=Trig（开漏输出 + 外部 10k 上拉，10us 高脉冲触发），PB4=Echo（输入，高电平宽度 = 距离），**非阻塞状态机驱动**，测量周期 500ms |
| 距离换算 | **毫米精度**：mm = Echo 高电平宽度(us) × 10 / 58（uint32 中间量防溢出），TIM1 1MHz 计数器提供 1us 级测量 |
| 数码管显示 | 两位八段**共阳**数码管，按距离分级显示：≥10cm 整数 cm（**个位带小数点 dp**，如 `12.`）、<10cm 带 1 位小数（点亮小数点 dp，如 `3.5` / `0.9`）、超量程（>99cm）或无效显示 `--` |
| UART 调试 | UART1 115200 8N1（PD5=TX / PD6=RX），打印启动信息与每次测距结果 |
| BEEP 提示 | 硬件 BEEP 外设（PD4，LSI 1/2/4kHz），有效测距短鸣 2kHz/50ms，超量程 1kHz/100ms，**无效/无回波不响** |

启动流程：初始化外设 -> 数码管显示 `--` -> 每 500ms 启动一次非阻塞测距（主循环
每轮调用 `service()` 推进状态机）-> 测量完成时更新显示 / 打印 UART / 播放提示音。

## 2. 接线表（TSSOP20）

| 信号 | 引脚 | 说明 |
|---|---|---|
| HC-SR04_TRIG | PB5 | 触发脉冲输出（**开漏 + 外部 10k 上拉到 VDD**），发 ≥10us 高电平 |
| HC-SR04_ECHO | PB4 | 回波输入（浮空）。**注意：HC-SR04 Echo 输出 5V，建议经 3.3k/2.2k 电阻分压再接 PB4** |
| UART1_TX | PD5 | 调试输出，115200 8N1 |
| UART1_RX | PD6 | 调试输入（可选，回显） |
| BEEP | PD4 | 硬件 BEEP 输出（LSI 时钟，1/2/4kHz） |
| SEG_a | PA1 | 数码管 a 段（共阳，经 220R 限流，**低有效**） |
| SEG_b | PC5 | 数码管 b 段 |
| SEG_c | PA3 | 数码管 c 段 |
| SEG_d | PC3 | 数码管 d 段 |
| SEG_e | PC4 | 数码管 e 段 |
| SEG_f | PA2 | 数码管 f 段 |
| SEG_g | PC6 | 数码管 g 段 |
| SEG_dp | PC7 | 小数点段 |
| DIG1 | PD2 | 第 1 位共阳（**高有效选中**） |
| DIG2 | PD3 | 第 2 位共阳 |
| VDD / VSS | 3/4 | 3.3V 供电，注意数码管段电流 |
| NRST | 2 | 复位（上拉 100nF 去耦） |

> 数码管为**两位共阳 8 段（a~g + dp）**：段输出低电平点亮（段码表为共阴表的
> 按位取反），位选高电平选中。全部数码管 GPIO（8 段 + 2 位）避开 HC-SR04
> （PB5/PB4）、UART（PD5/PD6）、BEEP（PD4）与 SWIM（PD1），无引脚冲突。
> TSSOP20 的 PD0 未引出，故只使用 10 个数码管 GPIO。

## 3. IO 资源评估（TSSOP20，20 脚）

| 功能 | 占用引脚 |
|---|---|
| HC-SR04 测距 | PB5（Trig，开漏 + 外部 10k 上拉）, PB4（Echo） |
| UART1 调试 | PD5, PD6 |
| BEEP 提示 | PD4 |
| 2 位数码管 | PA1, PA2, PA3, PC3, PC4, PC5, PC6, PC7, PD2, PD3 |
| 保留未用 | PD1（SWIM 调试口） |

HC-SR04 占用 PB4/PB5（原 I2C 引脚）；本工程不使用 I2C。

## 4. HC-SR04 测距原理与时序

HC-SR04 工作流程：

```
主控 (PB5)                 HC-SR04                   主控 (PB4)
  |-- Trig 高 >=10us ----->|                           |
  |                        |-- 发 8 个 40kHz 超声脉冲 ->| 障碍物
  |                        |<-- 回波 Echo 高电平 -------| 宽度 = 往返时间
  |-- 读 Echo 高电平宽度 ->|                           |
```

- **触发**：`Trig` 拉高 10us（代码 12us 余量），随后拉低；
- **回波**：`Echo` 高电平持续时间与距离成正比，**内部精度为毫米：
  距离(mm) = 高电平时间(us) × 10 / 58**（先乘 10 再除 58，使用 uint32
  中间量防溢出）；显示与 UART 仍以 cm 为单位输出；
- **时基**：TIM1 预分频 /16（HSI 16MHz -> 1MHz 计数，1us 分辨率），ARR=1000 产生
  1ms 更新中断；`g_tick`（1ms）与 TIM1 计数器（1us）合成单调微秒时钟用于测宽；
- **有效性**：回波宽度 116us（约 2cm）~ 30000us（约 5m）判定为有效，其余为
  超量程/无效；每 500ms 启动一次测量。
- **非阻塞状态机**：`start()` 发起一次测量（仅空闲 / 上次结果已消费时接受）；
  `service()` 由主循环每轮调用一次，状态机依次推进「等待 Echo 空闲 -> 触发
  10us 脉冲 -> 等待 Echo 高 -> 等待 Echo 低 -> 计算距离」，各等待均以 TIM1
  微秒时钟做超时保护（空闲 5ms / 回波 30ms / 下降沿 40ms），除 12us 触发脉冲
  忙等外不阻塞主循环；返回 `HCSR04_DONE` 表示本次完成，结果存于
  `valid` / `last_dist_mm`（毫米）。

## 5. 数码管显示策略

两位八段**共阳**数码管实时显示最近一次测距结果（内部毫米精度，显示换算为 cm）：

| 测量结果 | 显示 |
|---|---|
| 有效且 ≥ 100mm（≥10cm，≤99cm） | 整数 cm：十位 = mm/100（**不带 dp**）、个位 = (mm/10)%10 **并点亮小数点 dp**（如 123mm 显示 `12.`） |
| 有效且 < 100mm（<10cm） | 带 1 位小数的 cm：十位 = mm/10 **并点亮小数点 dp**、个位 = mm%10（如 35mm→`3.5`、9mm→`0.9`、5mm→`0.5`） |
| 有效但 > 990mm（>99cm） | `--`（超量程，两位显示不下三位数） |
| 无效 / 无回波 / 超时 | `--` |

- 小数点段 dp（PC7）在两种有效显示中被点亮：<10cm 时点亮十位（`mm/10`
  带 `SEG_FLAG_DP`，显示 `x.x`）；≥10cm 时点亮个位（`(mm/10)%10` 带
  `SEG_FLAG_DP`，显示 `xx.`）；其余显示不带小数点；
- 段码表为共阴表的按位取反（低电平点亮），位选高电平选中（与 project4 相反）；
- 动态扫描：TIM1 1ms ISR 中 `seg.onTime()` 每 1ms 点亮一位，2ms 一轮全刷新
  （约 500Hz/位，无闪烁）；
- 显示缓冲在 main 循环更新，段/位实际输出在 ISR 完成（非阻塞）。

## 6. BEEP 提示音

复用 project1 的 `beep.c/beep.h`：硬件 BEEP 外设（PD4），时钟源为内部 LSI
（128kHz），`BEEPDIV` 分频到 ~8kHz 后由 `BEEPSEL` 选择 1/2/4kHz 输出。
`beep.update()` 集成在 TIM1 1ms 时间片（自动关断定时）。

| 场景 | 提示音 |
|---|---|
| 有效测距（0..99cm） | 2kHz / 50ms 短鸣 |
| 有效但超量程（>99cm） | 1kHz / 100ms 低鸣 |
| 无效 / 无回波 / 超时 | 不响 |

> 无效测量（无回波、超时、回波宽度超范围）不播放提示音，避免无障碍物时蜂鸣器
> 持续鸣叫；仅有效结果给出提示。

## 7. UART 输出示例

```
STM8S103 HC-SR04 + 2x7SEG UART ready
Trig=PB5 Echo=PB4, period=500ms (non-blocking), mm precision, display cm
DIST=42 cm           (>=10cm: integer cm, display "42.", 420mm)
DIST=3.5 cm          (<10cm: one decimal, 35mm)
DIST=0.9 cm          (<10cm: one decimal, 9mm)
DIST=99 cm (OVR>99)  (>99cm: over-range, display "--")
DIST=-- cm (invalid / out of range)
...
```

## 8. 硬件设计（嘉立创EDA）

本工程包含由**嘉立创EDA（立创EDA）**设计的原理图与 PCB 工程文件：

| 文件 | 说明 |
|---|---|
| 雷达测距仪_2026-11-02.epro2 | 嘉立创EDA 工程文件（原理图 + PCB，约 250KB），可用嘉立创EDA 客户端/网页版导入查看与编辑 |

## 9. 文件清单

| 文件 | 说明 |
|---|---|
| main.c | 全局对象、时钟/TIM1 配置、ISR（TIM1 IRQ11、UART1_RX IRQ18）、1ms 时间片（数码管扫描 + BEEP 更新）、主循环非阻塞调度（500ms 周期启动测距 + 每轮调用 service） |
| hcsr04.h / .c | HC-SR04 非阻塞状态机驱动（start 触发 10us 脉冲、service 推进等待/计时/换算，超时保护），面向对象封装 |
| segled.h / .c | 两位共阳数码管动态扫描驱动（10 GPIO，段码表取反、位选高有效） |
| uart.h / .c | UART1 调试输出（环形缓冲 + 非阻塞 printf），与 project4 相同 |
| beep.h / .c | 硬件 BEEP 驱动（PD4，LSI 1/2/4kHz，定时自动关断），取自 project1 |
| Makefile / build.bat | SDCC 构建脚本（编译 -> 链接 -> packihx 出 hex） |
| README.md | 本文档 |
| 雷达测距仪_2026-11-02.epro2 | 嘉立创EDA 原理图 + PCB 工程（可用嘉立创EDA 导入查看/编辑） |

## 10. 编译与烧录

```bat
cd D:\stm8s_project\project5
build.bat                REM 或 make / make all
```

产物：`project5.ihx` / `project5.hex`。烧录：

```bat
D:\stm8s_project\stm8flash.exe -c stlinkv2 -p stm8s103f3 -w project5.ihx
```
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
*（内容由AI生成，仅供参考）*
