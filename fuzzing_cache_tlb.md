# DCache + DTLB Standalone Fuzzing DUT

## 概述

`StandaloneCacheWithTLB` 是一个独立的 RTL fuzzing 目标，包含：

- **EmbeddedTLB**（DTLB，64-entry Sv39）— 6 状态 PTW 状态机 + 权限检查
- **PTERequestFilter** — 过滤非法 PTE 地址（内置于 EmbeddedTLB.apply）
- **SimpleBusCrossbarNto1(2)** — 仲裁 TLB 翻译请求与 PTW 内存请求
- **DCache**（4-way 8KB）— 3 级流水线

相较于 `StandaloneCache`，TLB 的 PTW 状态机和页表遍历逻辑大幅增加了 mux/control
覆盖率达成难度，有助于发现更深层的硬件 bug。

## 文件位置

| 文件 | 说明 |
|------|------|
| `src/main/scala/top/CacheTLBGenTop.scala` | Scala wrapper + ChiselStage 入口 |
| `bmctest/emu/cache_tlb_tb.cpp` | Verilator C++ testbench（65B/cycle seed） |
| `Makefile` | `gen-cache-tlb` / `emu-cache-tlb` / `xfuzz-cache-tlb` |

## Seed 格式（65 B/cycle）

```
偏移  大小  字段           说明
 0    8B   vaddr          39-bit 虚拟地址（bit[38:0]有效）
 8    8B   wdata          写数据
16    1B   wmask          写字节掩码
17    1B   cmd            SimpleBusCmd（读/写/burst）
18    1B   size_val       传输大小 [2:0]
19    1B   ctrl           [0]=req_valid [1]=resp_ready [3:2]=flush
20    1B   env            [0..5]=mem/coh/mmio req_ready/resp_valid
21    8B   satp           SATP 寄存器（mode[63:60]/asid/ppn）
29    1B   mmu_ctrl       [1:0]=priviledgeMode [2]=sum [3]=mxr
                          [4]=flushTLB [5]=isAMO [6]=scInflight [7]=lr
30    8B   lr_addr        LR 地址
38    8B   mem_rdata      内存侧响应数据
46    1B   mem_cmd        内存侧响应命令
47    8B   coh_rdata      一致性侧响应数据
55    1B   coh_cmd        一致性侧响应命令
56    8B   mmio_rdata     MMIO 响应数据
64    1B   mmio_cmd       MMIO 响应命令
```

## BoringUtils 连接

**Wrapper 提供的 Sources（注入 TLB sinks）：**
- `CSRSATP` ← `io.satp`
- `MOUFlushTLB` ← `io.flushTLB`
- `ISAMO` ← `io.isAMO`
- `scInflight` ← `io.scInflight`
- `lr` ← `io.lr`
- `lr_addr` ← `io.lrAddr`

**Wrapper 提供的 Dummy Sinks（消化 DUT 内部 sources）：**
- `DTLBFINISH`, `DTLBPF`, `DTLBAF`, `vmEnable`, `dtlb_paddr`, `lsuMMIO`

## 构建流程

### 1. 生成插桩 RTL

```bash
# 默认覆盖率类型（mux,control）
make gen-cache-tlb

# 自定义覆盖率类型
make gen-cache-tlb CACHE_COVER=mux,control,toggle
```

生成文件：`build/modules/cache-tlb/StandaloneCacheWithTLB.sv`

### 2. 构建 Verilator 仿真器

```bash
# 不带覆盖率（支持快照保存/恢复，适合 BMC）
make emu-cache-tlb

# 带 Verilator 覆盖率
make emu-cache-tlb CACHE_COV=1
```

二进制：`build/emu-cache-tlb/emu-cache-tlb`

### 3. 构建 XFuzz/LibAFL 模糊测试器

```bash
# 需先构建 ccover: make xfuzz（或在 ccover/ 目录 cargo build --release）
make xfuzz-cache-tlb
```

二进制：`build/cache-tlb-xfuzz/cache-tlb-xfuzz`

## 运行示例

### 直接仿真（seed 驱动）

```bash
# 生成随机种子
dd if=/dev/urandom of=seed.bin bs=65 count=100

# 运行仿真（带 VCD 波形）
./build/emu-cache-tlb/emu-cache-tlb --seed seed.bin --cycles 5000 --vcd wave.vcd

# 保存快照
./build/emu-cache-tlb/emu-cache-tlb --seed seed.bin --cycles 1000 --snap-save snap.bin

# 从快照恢复继续
./build/emu-cache-tlb/emu-cache-tlb --seed seed.bin --snap-load snap.bin --cycles 5000
```

### XFuzz 模糊测试

```bash
# 准备初始 corpus
mkdir -p corpus/
dd if=/dev/urandom of=corpus/seed0.bin bs=65 count=10

# 启动 fuzzer
./build/cache-tlb-xfuzz/cache-tlb-xfuzz \
    -i corpus/ -o findings/ \
    --cover mux \
    -- --max-cycles 2000
```

## 与 StandaloneCache 对比

| 特性 | StandaloneCache | StandaloneCacheWithTLB |
|------|----------------|----------------------|
| TLB | 无 | EmbeddedTLB 64-entry Sv39 |
| 地址空间 | 32-bit 物理地址 | 39-bit 虚拟地址 |
| Seed 大小 | 46 B/cycle | 65 B/cycle |
| 额外状态 | — | PTW FSM(6-state) + ASID + 权限位 |
| 覆盖复杂度 | 基线 | 显著更高 |
