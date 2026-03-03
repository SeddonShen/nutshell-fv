```bash
# 1. 重新生成 RTL + firrtl-cover（加入 toggle）
make xfuzz-cache CACHE_COVER=mux,control,toggle

# 3. 生成种子
mkdir -p corpus && dd if=/dev/urandom of=corpus/seed0 bs=46 count=10


# 4. 运行 toggle fuzz
NOOP_HOME=$(pwd) COVER_POINTS_OUT=$(pwd)/tmp \
  ./build/cache-xfuzz/cache-xfuzz \
    -f --only-fuzz \
    -c firrtl.toggle \
    --corpus-input corpus/ \
    -- --max-cycles 2000
```

## 备注
mkdir -p corpus — 创建 corpus/ 目录（如果已存在则不报错）
dd if=/dev/urandom of=corpus/seed0 bs=46 count=10 — 从 /dev/urandom（系统随机数源）读取数据，写入 corpus/seed0 文件。bs=46 表示每块 46 字节，count=10 表示读 10 块，总共 460 字节。
为什么是 46？因为 cache_tb.cpp 中定义的 SeedCycle 结构体正好 46 字节——每个时钟周期的 DUT 输入。所以 460 字节 = 10 个周期的随机输入，作为 fuzzer 的初始种子语料。