---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: 495757a73352da7aae1b306638d4f5fe_790b1d22de8811f19232525400248c00
    ReservedCode1: AVYMRz2ViPqXjF3E/HRhfkmytOtdUS0cAeuvTyuWra/Ak1fYfPGWquIpP3OJdNsc96Yv7vC7NXrsgoKrrDBo7IJZFFPKnnn/dT7Wz2dhl0WT52zEwWXasZAsAI+T+WxF8r2u7td6g0LNKy4pohhHvjD7NeeGofqjFFfaF0wKxTqCS8YEfsE3AC5XV2M=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: 495757a73352da7aae1b306638d4f5fe_790b1d22de8811f19232525400248c00
    ReservedCode2: AVYMRz2ViPqXjF3E/HRhfkmytOtdUS0cAeuvTyuWra/Ak1fYfPGWquIpP3OJdNsc96Yv7vC7NXrsgoKrrDBo7IJZFFPKnnn/dT7Wz2dhl0WT52zEwWXasZAsAI+T+WxF8r2u7td6g0LNKy4pohhHvjD7NeeGofqjFFfaF0wKxTqCS8YEfsE3AC5XV2M=
---









# STM8S103F3P6 project5 — HC-SR04 超声波测距 + 2 位 7 段数码管显示 + UART 调试 + BEEP

本工程基于 **STM8S103F3P6（TSSOP20）**，采用 **SDCC** 编译，工程结构与编码风格与
`project4` 完全一致：面向对象封装（结构体 + 函数指针）、`main.c` 集中定义 ISR、
TIM1 1ms 时间片调度、数码管动态扫描、UART 调试输出。与 project4 的关键区别：

- **外设更换**：去掉 I2C（不再使用 PB4/PB5），新增 **HC-SR04 超声波测距**；
- **数码管更换**：四位共阴改为**两位共阳**（段码表取反为低电平点亮、位选高电平
  选中，位选只需 2 个）；
- **新增 BEEP**：复用 project1 的 `beep.c/beep.h`（硬件 BEEP 外设 PD4，LSI 时钟，
  1/2/4kHz），**两级报警：<10cm 每次采纳读数都短鸣 1 次，10~20cm 每 2 次短鸣 1 次**
  （≥20cm、无效 / 无回波不响）。

- MCU: STM8S103F3P6（TSSOP20）
- 时钟: HSI 16MHz（无外部晶振）
- 编译器: SDCC 4.5.0（`sdcc -mstm8 --std-c99`）
- 烧录: stm8flash（ST-LINK/V2）

## 1. 功能概述

| 功能 | 说明 |
|---|---|
| HC-SR04 测距 | PB5=Trig（开漏输出 + 外部 10k 上拉，10us 高脉冲触发），PB4=Echo（输入，高电平宽度 = 距离），**非阻塞状态机驱动**，测量周期 200ms |
| 距离换算 | **毫米精度**：mm = Echo 高电平宽度(us) × 10 / 58（uint32 中间量防溢出），TIM1 1MHz 计数器提供 1us 级测量 |
| 数码管显示 | 两位八段**共阳**数码管，显示量程上限 **1m**：≥10cm 且 <100cm 整数 cm（**个位带小数点 dp**，如 `12.`）、<10cm 带 1 位小数（点亮小数点 dp，如 `3.5` / `0.9`）、**≥1m 或无效时一律显示 `--`** |
| UART 调试 | UART1 115200 8N1（PD5=TX / PD6=RX），打印启动信息与每次测距结果 |
| BEEP 提示 | 硬件 BEEP 外设（PD4，LSI 1/2/4kHz），**两级报警**：**<10cm 每次采纳读数都短鸣 1 次**（200ms 一次，~5Hz 急促）；**10~20cm 每累计 2 次采纳读数短鸣 1 次**（400ms 一次）；均为 2kHz/50ms。**≥20cm（含 20cm~1m 的显示区间）、≥1m、无效/无回波一律不响** |

启动流程：初始化外设 -> 数码管显示 `--` -> 每 200ms 启动一次非阻塞测距（主循环
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

- **触发**：`Trig` 拉高 20us（数据手册要求 >=10us，代码给 20us 余量），随后拉低；
  本板 Trig 由 3.3V 开漏引脚驱动、依靠外部 10k 上拉，而 HC-SR04 输入门限约
  3.0~3.5V，3.3V 恰好卡在门限边缘，脉宽过短时传感器约有一半概率漏触发
  （表现为大量 `us=0`），因此把脉宽放宽到 20us 以穿越上拉上升沿与门限不确定性；
- **回波**：`Echo` 高电平持续时间与距离成正比，**内部精度为毫米：
  距离(mm) = 高电平时间(us) × 10 / 58**（先乘 10 再除 58，使用 uint32
  中间量防溢出）；显示与 UART 仍以 cm 为单位输出；
- **时基**：TIM1 预分频 /16（HSI 16MHz -> 1MHz 计数，1us 分辨率），ARR=1000 产生
  1ms 更新中断；`g_tick`（1ms）与 TIM1 计数器（1us）合成单调微秒时钟用于测宽；
- **有效性**：回波宽度 134us（约 2.3cm）~ 30000us（约 5m，`HCSR04_MAX_US` 保持
  30000 不变）判定为有效，其余为超量程/无效；每 200ms 启动一次测量。
  **< 134us（约 2.3cm 以下）落在 HC-SR04 声学盲区内、实测不准，一律判无效**：
  显示两位横杠 `--`、不蜂鸣、UART 打印 `invalid`（`us=` 后仍给出原始宽度）。
  注意区分两个「量程」：**传感器有效量程约 2.3cm~5m**（决定 `valid`），
  **显示量程上限为 1m**（`valid` 但 ≥1m 时显示 `--`，UART 仍打印真实回波宽度，
  见第 5 节），**蜂鸣只在 <20cm 触发，其中 <10cm 升级为每次必响**（见第 6 节）。
- **失败自动重试**：一个 200ms 周期内，若某次尝试结果无效（`valid==0` 或
  `last_echo_us==0`），间隔 20ms 后自动重新 `start()`，最多重试 2 次（每周期
  最多 3 次尝试），任一次成功即停止重试；重试计数与间隔均基于 `g_tick`，不阻塞
  主循环；3 次全失败仍按原逻辑显示 `--` 且不蜂鸣。失败尝试不会立即刷新显示/
  串口，只有最终结果（成功或 3 次全败）才上报。
- **非阻塞状态机**：`start()` 发起一次测量（仅空闲 / 上次结果已消费时接受）；
  `service()` 由主循环每轮调用一次，状态机依次推进「等待 Echo 空闲 -> 触发
  20us 脉冲 -> 等待 Echo 高 -> 等待 Echo 低 -> 下降沿去抖确认 -> 计算距离」，
  各等待均以 TIM1 微秒时钟做超时保护（空闲 5ms / 回波 30ms / 下降沿 40ms），
  除 20us 触发脉冲忙等外不阻塞主循环；返回 `HCSR04_DONE` 表示本次完成，结果存于
  `valid` / `last_dist_mm`（毫米）。
- **Echo 下降沿去抖确认（200us 窗口）**：`WAIT_LOW` 首次采到低电平不直接当作
  下降沿，只把该时刻记为**候选** `t1` 并进入 `WAIT_CONFIRM`；`HCSR04_CONFIRM_US`
  = 200us 后重采样，仍为低才确认是真实下降沿，若已回到高则判为毛刺丢弃并退回
  `WAIT_LOW` 继续等待真正下降沿。真实下降沿之后 Echo 会持续低电平数毫秒（直到
  下一个 200ms 触发），因此 200us 窗口不会误伤真沿；而远端测距时叠加在宽回波
  上的短时低电平干扰无法维持 200us，会被滤除。测宽仍使用**候选时刻** `t1` 与上升
  沿 `t0` 之差，确认延迟不引入误差。确认态同时设时间条件与重试上限
  （`HCSR04_CONFIRM_MAX_RETRY` = 1500 次 `service()` 调用，仅在微秒时钟停摆时
  兜底），200us 窗口正常只需约 40~70 次调用，上限不会提前截断窗口。

## 5. 数码管显示策略

两位八段**共阳**数码管实时显示最近一次测距结果（内部毫米精度，显示换算为 cm）：

| 测量结果 | 显示 |
|---|---|
| 有效且 ≥ 100mm 且 < 1000mm（≥10cm，<1m） | 整数 cm，**四舍五入到最近 cm**：十位 = r/100（**不带 dp**）、个位 = (r/10)%10 **并点亮小数点 dp**，其中 `r = mm + 5`（如 123mm→`12.`、148mm→`15.`、996mm→`99.`，不进位成 `100`） |
| 有效且 < 100mm（2.3cm ~ 9.9cm） | 带 1 位小数的 cm：十位 = mm/10 **并点亮小数点 dp**、个位 = mm%10（如 35mm→`3.5`、23mm→`2.3`） |
| 有效且 ≥ 1000mm（≥1m） | `--`（**超出显示量程**：两位都显示横杠、不带 dp，与无效读数外观一致；真实测距值仍通过 UART 打印） |
| 回波宽度 < 134us（< 约 2.3cm，盲区内） | `--`（按无效处理，不蜂鸣，UART 打印 invalid 并附原始 `us`） |
| 无效 / 无回波 / 超时 | `--` |

- 小数点段 dp（PC7）仅在**有效且 <1m** 的显示中被点亮：<10cm 时点亮十位
  （`mm/10` 带 `SEG_FLAG_DP`，显示 `x.x`）；≥10cm 且 <100cm 时点亮个位
  （`(mm/10)%10` 带 `SEG_FLAG_DP`，显示 `xx.`）；**≥1m 与无效 / 无回波显示
  `--`，均不带小数点**；
- 段码表为共阴表的按位取反（低电平点亮），位选高电平选中（与 project4 相反）；
- 动态扫描：TIM1 1ms ISR 中 `seg.onTime()` 每 1ms 点亮一位，2ms 一轮全刷新
  （约 500Hz/位，无闪烁）；
- 显示缓冲在 main 循环更新，段/位实际输出在 ISR 完成（非阻塞）。

### 5.1 距离跳变抑制

量程极限处弱回波偶尔返回明显偏短的宽度（真实 439cm 读到 124/125cm），会让显示
在单个周期内跳变数米。读数在进入显示 / 蜂鸣前先经 `dist_filter()` 过滤：

- 首个读数，或与上次采纳值的偏差 ≤ `HCSR04_JUMP_THRESHOLD_MM`（默认 300mm）
  → 立即采纳；
- 偏差大于阈值、但与上一条未确认离群值偏差同样 ≤ 阈值 → 判定目标确实移动，
  采纳**上一条离群值**（保持显示值落在已确认的簇内）；
- 以上都不满足 → 记为待确认离群值，显示保持上一值、不蜂鸣，UART 打印
  `(suppressed: jump)`；
- 待确认离群值连续 `HCSR04_PENDING_MAX`（默认 3）个周期被拒 → 强制采纳最新值；
- 无效读数（`us=0` 或 `valid=0`）不参与跳变判断，直接走无效分支。

### 5.2 中位数滤波

有效读数在进入 `dist_filter()` 之前先过一级滑动中位数（`median_push()`），窗口
`HCSR04_MEDIAN_WIN`（默认 **3** 点，每个测量周期 200ms 采样一次）：

- 剔除单周期孤立尖峰（上一周期残留回波、电磁干扰引起的单点跳变）；
- 代价是跟随延迟约为 `(WIN-1)/2` 个周期：WIN=3 → 约 200ms，WIN=5 → 约
  400ms。窗口越大越稳，但目标缓慢移动时中位数会被旧样本"拖住"（原来的 5 点
  窗口在缓慢靠近时表现为读数长时间不变、随后一次跳两格）；
- 窗口首次填满前的读数取已有样本的中位数，不做额外等待；
- 超量程（≥1m）分支会调用 `median_reset()` 清空窗口，重回量程时首读直接采纳。

### 5.3 显示滞回

`display_update()` 只在「本次采纳值」与「面板当前值」之差 ≥
`HCSR04_DISPLAY_HYST_MM`（默认 **10**，即一个最低显示位）时才刷新面板，
否则保持原值并打印 `HOLD`：

- 滞回必须 ≤ 1cm（即一个显示步进）：否则 10 个 mm 的差值内面板不刷新，
  缓慢移动时会出现"面板停住、随后跨窗跳两格"的台阶感；
- 与 §5 的四舍五入配合：面板显示值与真实值偏差始终 < 5mm + 半格；
- 静止时 ±4cm 级别的噪声由中位数 + 跳变抑制负责，不依赖大滞回；
- 噪声环境偏大时可上调到 20~30，但会以"台阶跟随"换取面板稳定。

## 6. BEEP 提示音

复用 project1 的 `beep.c/beep.h`：硬件 BEEP 外设（PD4），时钟源为内部 LSI
（128kHz），`BEEPDIV` 分频到 ~8kHz 后由 `BEEPSEL` 选择 1/2/4kHz 输出。
`beep.update()` 集成在 TIM1 1ms 时间片（自动关断定时）。

| 场景 | 提示音 |
|---|---|
| 采纳的有效读数 < 100mm（<10cm） | **紧急档：每次采纳读数都短鸣 1 次**（200ms 一次，~5Hz），2kHz / 50ms；节流计数器被钉在 0 |
| 采纳的有效读数 100~200mm（10~20cm） | 2kHz / 50ms 短鸣，**每累计 2 次采纳的近距读数才响 1 次**（`HCSR04_NEAR_BEEP_EVERY`），响后计数器清零（400ms 一次） |
| 采纳的有效读数 ≥ 200mm（含 20cm~1m 的显示区间） | 不响，并**清零近距计数器** |
| 有效但 ≥1m（超出显示量程） | 不响 |
| 被跳变抑制、未被采纳的读数 | 不响、不计数 |
| 无效 / 无回波 / 超时 | 不响，并**清零近距计数器** |

> 蜂鸣采用**两级升档**，凭听感即可判断距离：<10cm 是 200ms 一次的连续急促报警
> （~5Hz），10~20cm 是 400ms 一次的提醒（每 2 次采纳读数响 1 次）；两档响度与
> 频率相同，只靠节奏区分，避免用户为了区分而额外调频。
> 蜂鸣范围（<20cm）比显示范围（<1m）更窄：20cm~1m 区间只显示数值、不鸣叫，
> 避免中距离持续响。一旦读数不再小于 20cm（≥20cm）或转为无效，近距计数器立即
> 清零，避免残留计数在下次靠近时提前触发；进入 <10cm 紧急档时计数器同样钉在 0，
> 退回 10~20cm 档后从 0 重新计数。`HCSR04_CRIT_MM`（默认 100）、
> `HCSR04_NEAR_MM`（默认 200）与 `HCSR04_NEAR_BEEP_EVERY`（默认 2）均在
> `main.c` 顶部可改。

## 7. UART 输出示例

```
STM8S103 HC-SR04 + 2x7SEG UART ready
Trig=PB5 Echo=PB4, period=200ms (non-blocking, retry x2), mm precision, display cm
DIST=42 cm (us=2440)
DIST=3.5 cm (us=203)
DIST=2.3 cm (us=134)                   (~2.3cm: lowest accepted width, display "2.3")
DIST=-- cm (OVR>=1m, us=25523)         (>=1m: out of display range, display "--", no beep)
DIST=-- cm (invalid / out of range, us=116)  (us<134: blind zone, invalid, display "--", no beep)
DIST=-- cm (invalid / out of range, us=0)
DIST=124 cm (suppressed: jump) us=7192 (unconfirmed jump: display keeps last value, no beep)
DIST=4.3 cm (suppressed: jump) us=250  (same rule for sub-10cm readings)
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
| main.c | 全局对象、时钟/TIM1 配置、ISR（TIM1 IRQ11、UART1_RX IRQ18）、1ms 时间片（数码管扫描 + BEEP 更新）、主循环非阻塞调度（200ms 周期启动测距 + 每轮调用 service） |
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
*（内容由AI生成，仅供参考）*
