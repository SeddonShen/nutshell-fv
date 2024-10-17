# 一些有用的命令

```bash
./build/fuzzer -f --corpus-input /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -c firrtl.toggle -- /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -e 0 --no-diff > 4716_debug.log

./build/fuzzer -c firrtl.toggle -- /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -e 0 --no-diff

(不-f不会有)
./build/fuzzer -f --max-runs 1000 --corpus-input /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/ -c firrtl.toggle -- --no-diff -I 100 -e 0 --max-cycles 500> ssd1k_help_only_hexbin.log
```

## 编译命令

```bash
make clean && make emu REF=$SPIKE_HOME/difftest/build/riscv64-spike-so XFUZZ=1 FIRRTL_COVER=toggle -j16
```

### 直接编译

```bash
export NOOP_HOME=$(pwd) 
make clean && make emu
```

### 简单的跑指令调试