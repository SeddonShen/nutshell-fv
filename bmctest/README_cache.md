# NutShell Cache — BMCFuzz 独立模块适配

## 概述

从 NutShell 处理器中提取 **dcache** 模块，封装为独立的 `StandaloneCache`，在 BMCFuzz 框架下进行形式验证和混合模糊测试。

与 `bmctest/` 目录中已有的 15 个 rocket-chip 模块共享同一测试基础设施（`bmc_depth_test.py`、FormalTop 包装、sby 任务生成），同时新增 Verilator EMU 驱动以支持 BMCFuzz 的快照机制。

---

## 模块架构

NutShell dcache 是一个 **3 级流水线**缓存：

```
CPU (SimpleBusUC, userBits=16)
        │
  ┌─────▼─────┐
  │  Stage 1   │  地址译码、Tag/Data SRAM 读请求
  ├───────────┤
  │  Stage 2   │  Tag 比对、命中/缺失判定
  ├───────────┤
  │  Stage 3   │  数据写回、缺失处理、MMIO 旁路
  └─────┬─────┘
        │
  ┌─────▼─────┐
  │  Memory    │  SimpleBusC (mem + coh)
  │  + MMIO    │  SimpleBusUC
  └───────────┘
```

### IO 端口（CacheIO）

| 端口 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `io_in` | `SimpleBusUC(userBits=16)` | CPU→Cache | CPU 侧请求/响应 |
| `io_flush` | `UInt(2.W)` | Input | 流水线 flush 控制 |
| `io_out.mem` | `SimpleBusUC` | Cache→Memory | 内存侧读写请求 |
| `io_out.coh` | `SimpleBusUC` | Cache↔Memory | 一致性通道 |
| `io_mmio` | `SimpleBusUC` | Cache→MMIO | MMIO 旁路 |
| `io_empty` | `Bool` | Output | 流水线空指示 |

### SimpleBus 信号

**请求通道** (Decoupled)：`valid`, `ready`, `bits.addr[31:0]`, `bits.size[2:0]`, `bits.cmd[3:0]`, `bits.wmask[7:0]`, `bits.wdata[63:0]`, `bits.user[15:0]`

**响应通道** (Flipped Decoupled)：`valid`, `ready`, `bits.cmd[3:0]`, `bits.rdata[63:0]`, `bits.user[15:0]`

---

## CacheConfig 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ro` | Boolean | `false` | 只读模式（icache 用 true，dcache 用 false） |
| `name` | String | `"dcache"` | 缓存实例名称 |
| `userBits` | Int | `16` | SimpleBus user 字段宽度（`DCacheUserBundleWidth`） |
| `idBits` | Int | `0` | SimpleBus ID 字段宽度 |
| `cacheLevel` | Int | `1` | 缓存层级 |
| `totalSize` | Int | `256` | 总容量（KB）—— 形式验证时可考虑缩小至 4KB |
| `ways` | Int | `2` | 组相联路数 |

运行时参数来自 `DefaultSettings()`：

| 参数 | 值 | 说明 |
|------|----|------|
| `XLEN` | 64 | 数据宽度 |
| `PAddrBits` | 32 | 物理地址宽度 |
| `EnableOutOfOrderExec` | false | 关闭乱序执行（不触发性能计数器 BoringUtils） |
| `MMIOBase` | 0x40000000 | MMIO 起始地址 |
| `MMIOSize` | 0x40000000 | MMIO 区域大小 |

---

## 构建与测试流程

### 前置依赖

| 工具 | 用途 |
|------|------|
| Mill | Scala 构建（Chisel 3.6.1） |
| Verilator (≥4.228) | C++ 仿真编译 |
| SymbiYosys (sby) | 形式验证前端 |
| bitwuzla | SMT 求解器（包含在 OSS CAD Suite 中） |
| rIC3 | SAT-based IC3/PDR 引擎（位于 `ccover/Formal/bin/rIC3`） |
| Python 3.8+ | BMC 测试脚本，可选依赖 `tqdm` |

### Step 1：生成 SystemVerilog

```bash
make gen-cache CACHE_COVER=toggle
```

产物位于 `build/modules/cache/`：
- `StandaloneCache.sv` — 主模块
- `GEN_w*_*.v` — 覆盖率插桩黑盒
- `plusarg_reader.v` — Chisel 辅助模块

### Step 2：BMC 深度测试

```bash
# SMT 模式（cover + smtbmc/bitwuzla）
python3 bmctest/bmc_depth_test.py \
    --module cache --depth 50 --timeout 3600 --mode smt

# SAT 模式（bmc + aiger/rIC3）
python3 bmctest/bmc_depth_test.py \
    --module cache --depth 50 --timeout 3600 --mode sat \
    --ric3 ccover/Formal/bin/rIC3
```

结果输出到 `bmctest/work/cache/results.json`。

### Step 3：Verilator EMU 编译

```bash
# 基础编译（含 VCD trace + snapshot）
make emu-cache

# 含覆盖率（toggle/line/branch，会降低仿真速度）
make emu-cache CACHE_COV=1
```

产物：`build/emu-cache/emu-cache`

### Step 4：种子驱动仿真

```bash
# 基本仿真
./build/emu-cache/emu-cache --seed seed.bin --cycles 5000

# 带 VCD 波形
./build/emu-cache/emu-cache --seed seed.bin --cycles 5000 --vcd wave.vcd

# 保存快照（用于 BMC 恢复）
./build/emu-cache/emu-cache --seed seed.bin --cycles 5000 --snap-save snap.bin

# 从快照恢复（跳过 reset，从中间状态继续）
./build/emu-cache/emu-cache --seed seed2.bin --snap-load snap.bin --cycles 3000
```

### EMU 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--seed <file>` | (必需) | 二进制种子文件 |
| `--cycles <N>` | 10000 | reset 后仿真周期数 |
| `--reset-cycles <N>` | 10 | reset 持续周期数 |
| `--snap-save <file>` | — | 仿真结束后保存快照 |
| `--snap-load <file>` | — | 仿真前加载快照（跳过 reset） |
| `--cov-out <file>` | coverage.dat | 覆盖率数据输出路径 |
| `--vcd <file>` | — | VCD 波形输出路径 |

---

## 种子格式

种子文件是一段平坦的二进制字节流。每 **46 字节**映射为一个时钟周期的 DUT 输入信号：

```
偏移   大小   信号
 0     4B    io_in_req_bits_addr    [31:0]
 4     8B    io_in_req_bits_wdata   [63:0]
12     2B    io_in_req_bits_user    [15:0]
14     1B    io_in_req_bits_wmask   [7:0]
15     1B    io_in_req_bits_cmd     [3:0]
16     1B    io_in_req_bits_size    [2:0]
17     1B    ctrl — bit 0: req_valid
                    bit 1: resp_ready
                    bit 3:2: flush
18     1B    env  — bit 0: mem_req_ready
                    bit 1: mem_resp_valid
                    bit 2: coh_req_ready
                    bit 3: coh_resp_valid
                    bit 4: mmio_req_ready
                    bit 5: mmio_resp_valid
19     8B    io_out_mem_resp_bits_rdata
27     1B    io_out_mem_resp_bits_cmd
28     8B    io_out_coh_resp_bits_rdata
36     1B    io_out_coh_resp_bits_cmd
37     8B    io_mmio_resp_bits_rdata
45     1B    io_mmio_resp_bits_cmd
───────────
合计   46B / cycle
```

种子不足时自动循环复用。

---

## BMCFuzz 混合测试数据流

```
                  seed (SimpleBus transactions)
                            │
                            ▼
                  ┌───────────────────┐
                  │ cache_tb (C++)    │
                  │ ● 驱动 SimpleBus  │
                  │ ● 收集覆盖率      │
                  │ ● 保存快照        │
                  └────────┬──────────┘
                           │
                  ┌────────▼──────────┐
                  │ StandaloneCache   │
                  │ (Verilated)       │
                  └────────┬──────────┘
                           │
                  coverage bitmap + snapshots
                           │
                  ┌────────▼──────────┐
                  │ BMC (sby)         │
                  │ ● 从快照状态出发  │
                  │ ● 搜索未覆盖点    │
                  │ ● 生成 witness    │
                  └────────┬──────────┘
                           │
                      new seeds
                     (反馈到 fuzzer)
```

核心思想：Fuzz 快速探索浅层状态空间 → 快照捕获中间状态 → BMC 从快照出发搜索深层覆盖点，两者交替迭代。

---

## 与 rocket-chip 模块的对比

| 特性 | NutShell Cache | rocket-chip 模块 |
|------|---------------|-----------------|
| 总线协议 | SimpleBus | TileLink |
| 拓扑 | 独立 Module | Diplomacy LazyModule（含 TLFuzzer + TLRAM） |
| SV 规模 | ~5K–10K 行（预计） | 67 行 (ecc) ~ 112K 行 (toaxi4) |
| 状态复杂度 | 中等（3 级流水 + SRAM） | 小到大不等 |
| BMC 瓶颈 | 预计有（SRAM 状态空间大） | 部分模块有瓶颈 |
| FormalTop | `bmctest/formal_top/cache/` | `bmctest/formal_top/<name>/` |
| 覆盖率插桩 | `xfuzz.CoverPoint` (Chisel 3.6.1) | 同左 |

NutShell Cache 因其 SRAM 阵列（默认 256KB, 2-way）导致状态空间较大，是 BMCFuzz 的理想 benchmark：纯 BMC 难以在浅深度覆盖所有点，但快照机制能有效缩短搜索路径。

---

## 关键文件路径

```
nutshell-fv/
├── src/main/scala/top/
│   └── CacheGenTop.scala             # StandaloneCache wrapper + CacheGenMain
├── build/modules/cache/              # gen-cache 产物
│   ├── StandaloneCache.sv
│   ├── GEN_w*_*.v
│   └── plusarg_reader.v
├── build/emu-cache/                  # emu-cache 产物
│   ├── emu-cache                     # Verilator 仿真二进制
│   └── obj/                          # Verilator 编译中间文件
├── bmctest/
│   ├── README_cache.md               # 本文件
│   ├── bmc_depth_test.py             # BMC 深度测试脚本
│   ├── emu/
│   │   └── cache_tb.cpp              # Verilator C++ 测试驱动
│   ├── formal_top/cache/
│   │   └── FormalTop.sv              # BMC 形式验证顶层包装
│   └── work/cache/                   # 运行时工作目录（自动生成）
│       ├── rtl/                      # 插桩后的 RTL
│       ├── tasks/                    # SBY 任务及输出
│       └── results.json              # BMC 结果报告
└── Makefile                          # gen-cache / emu-cache 目标
```

---

## 注意事项

1. **FormalTop 端口校准**：`FormalTop.sv` 中的端口名基于 Chisel 命名约定预测，已在 SV 生成后从实际文件中确认。若 CacheConfig 参数修改导致端口变化，需同步更新 FormalTop。

2. **BoringUtils 兼容**：dcache 的 `CacheStage3` 调用 `BoringUtils.addSource(mmio, "lsuMMIO")`，独立模式下无 LSU 消费该信号。`StandaloneCache` 中已添加 `BoringUtils.addSink(dummyLsuMMIO, "lsuMMIO")` 解决配对问题。

3. **CacheConfig.totalSize**：默认 256KB，生成的 SRAM 较大。若形式验证工具内存不足，可在 `CacheGenTop.scala` 中调小 `totalSize`（如 4KB），但需重新生成 SV 和 FormalTop。

4. **AddressSpace.isMMIO**：依赖 `Settings.getLong("MMIOBase")` 和 `MMIOSize`。`DefaultSettings` 中已有默认值（0x40000000），无需修改。

5. **Verilator 版本**：需 ≥4.228 以支持 `VerilatedContext` API 和 `--savable` 标志。推荐 Verilator 5.x。
