# 一些有用的命令

```bash
./build/fuzzer -f --corpus-input /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -c firrtl.toggle -- /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -e 0 --no-diff > 4716_debug.log

# 单独统计某一个种子的覆盖率
./build/fuzzer -c firrtl.toggle -- /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -e 0 --no-diff --max-cycles 500

(不-f不会有)
./build/fuzzer -f --max-runs 1000 --corpus-input $CORPUS -c firrtl.toggle -- --no-diff -I 100 -e 0 --max-cycles 500> ssd1k.log
./build/fuzzer -f --max-runs 1000 --corpus-input /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/ -c firrtl.toggle -- --no-diff -I 100 -e 0 --max-cycles 500> ssd1k_help_only_hexbin.log

./build/fuzzer -f --max-runs 100 --corpus-input $CORPUS -c firrtl.toggle -- --max-cycles 10000 > test.log

# 将XFuzz结果输出到文件中
./build/fuzzer -f --formal-cover-rate 500.0 --corpus-input $CORPUS_DIR --cover-points-output $FUZZ_COVER_POINTS_OUT -c firrtl.toggle -- --no-diff -I 100 -e 0
```

## 编译命令

```bash
# 编译为EMU
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so -j16
# 编译为Fuzzer
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so XFUZZ=1 FIRRTL_COVER=toggle -j16
```

### 直接编译

```bash
export NOOP_HOME=$(pwd) 
make clean && make emu
```

### 简单的跑指令调试

```bash
# 带difftest
./build/emu -i ./ready-to-run/microbench.bin
# 不带difftest
./build/emu -i ./ready-to-run/microbench.bin --no-diff
```