# 一些有用的命令

## 如何运行

```bash
source env.sh
# 编译xfuzz
make xfuzz
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so XFUZZ=1 FIRRTL_COVER=toggle -j16

# 运行,暂时没添加参数，可以手动修改run函数里的参数
python ./ccover/Formal/Scheduler.py
```

## xfuzz命令 

```bash
./build/fuzzer -f --corpus-input /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -c firrtl.toggle -- /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -e 0 --no-diff > 4716_debug.log

# 单独统计某一个种子的覆盖率
./build/fuzzer -c firrtl.toggle -- /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/cover_4716.bin -e 0 --no-diff --max-cycles 500

(不-f不会有)
./build/fuzzer -f --max-runs 1000 --corpus-input $CORPUS -c firrtl.toggle -- --no-diff -I 100 -e 0 --max-cycles 500> ssd1k.log
./build/fuzzer -f --max-runs 1000 --corpus-input /home/seddon/Coding/formal_fuzzing/CoverCount/coverTasks/hexbin/ -c firrtl.toggle -- --no-diff -I 100 -e 0 --max-cycles 500> ssd1k_help_only_hexbin.log

./build/fuzzer -f --max-runs 100 --corpus-input $CORPUS -c firrtl.toggle -- --max-cycles 10000 > test.log

# 将XFuzz结果输出到文件中
./build/fuzzer -f --formal-cover-rate 0.0659 --corpus-input $CORPUS_DIR --cover-points-output $COVER_POINTS_OUT -c firrtl.toggle -- -I 100 -e 0
./build/fuzzer -f --formal-cover-rate 0.0659 --corpus-input $CORPUS_DIR --cover-points-output $COVER_POINTS_OUT -c firrtl.toggle -- --no-diff -I 100 -e 0
./build/fuzzer -f --corpus-input $CORPUS_DIR --cover-points-output $COVER_POINTS_OUT -c firrtl.toggle -- --no-diff -I 100 -e 0
./build/fuzzer -f --max-iters 1 --corpus-input $CORPUS_DIR --cover-points-output $COVER_POINTS_OUT -c firrtl.toggle -- --no-diff -I 100 -e 0
./build/fuzzer -c firrtl.toggle -- tmp/bin/test.bin -I 100 -e 0 
./build/fuzzer -f --max-runs 10 --corpus-input $CORPUS_DIR -c firrtl.toggle -- -I 100 -C 500 -b 0 --dump-ref-trace --dump-commit-trace > tmp/test.log 2>&1
./build/fuzzer -f --max-runs 10 --corpus-input $CORPUS_DIR -c firrtl.toggle --insert-nop -- -I 100 -C 500 -b 0 --dump-wave-full > tmp/test.log 2>&1
./build/fuzzer -c firrtl.toggle -- tmp/bin/test.bin -I 1000 -b 0 --dump-ref-trace --dump-commit-trace > tmp/fuzz.log 2>&1
./build/fuzzer -c firrtl.toggle -- tmp/bin/test.bin --no-diff -I 1000 -b 0 --dump-wave-full
./build/fuzzer -c firrtl.toggle -- tmp/bin/switch_mode_toU.bin  -I 200 -C 500 -b 0 --dump-wave-full > tmp/test.log
./build/fuzzer -c firrtl.toggle -- tmp/bin/switch_mode_toU.bin  -I 200 -C 500 -b 0 --snapshot-cycles 56 --snapshot-image tmp/bin/snapshot.bin  > tmp/test.log
./build/fuzzer -f --max-iters 1200 --corpus-input $RISCV_CORPUS --cover-points-output $COVER_POINTS_OUT -c firrtl.toggle -- --no-diff -I 1000 -C 10000 -e 0 > tmp/test.log


./build/fuzzer -c firrtl.toggle -- tmp/bin/test.bin  -I 200 -C 500 -b 0 --snapshot-cycles 62 --snapshot-image tmp/bin/snapshot.bin  > tmp/test.log

./build/fuzzer -f --formal-cover-rate 0.0001 --corpus-input $CORPUS_DIR -c firrtl.toggle --insert-nop --continue-on-errors --run-snapshot --snapshot-file /home/chooaa/HW_formal_verification/nutshell-fv/ccover/SetInitValues/csr_snapshot/3 -- -C 10000 -b 0 --snapshot-cycles 76  > tmp/fuzz.log 2>&1

./build/fuzzer -c firrtl.toggle -- tmp/bin/test.bin -C 10000 -b 0 --snapshot-cycles 87 --fuzz-id 0 --dump-csr-change --dump-wave-full --wave-path tmp/run_wave.vcd --snapshot-image tmp/bin/snapshot.bin > fuzz.log 2>&1

./build/fuzzer -c firrtl.toggle -- tmp/bin/test.bin -C 10000 -b 0 --fuzz-id 1 --dump-csr-change --dump-wave-full --wave-path tmp/run_wave.vcd > tmp/fuzz.log 2>&1

```

## 编译命令

```bash
# 编译为EMU
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so -j16
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so EMU_TRACE=1 -j16
# 编译为Fuzzer
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so XFUZZ=1 FIRRTL_COVER=toggle -j16
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so XFUZZ=1 FIRRTL_COVER=toggle EMU_TRACE=1 -j16
make clean && make src REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so XFUZZ=1 FIRRTL_COVER=toggle EMU_TRACE=1 -j16
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-spike-so XFUZZ=1 FIRRTL_COVER=toggle EMU_TRACE=1 -j16
make fuzzer REF=$(pwd)/ready-to-run/riscv64-nemu-interpreter-so XFUZZ=1 FIRRTL_COVER=toggle EMU_TRACE=1 -j16

# spike
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-spike-so EMU_TRACE=1 -j16
make clean && make emu REF=$(pwd)/ready-to-run/riscv64-spike-so XFUZZ=1 FIRRTL_COVER=toggle EMU_TRACE=1 -j16

make clean && make src REF=$(pwd)/ready-to-run/riscv64-spike-so XFUZZ=1 FIRRTL_COVER=toggle EMU_TRACE=1 -j16
make fuzzer REF=$(pwd)/ready-to-run/riscv64-spike-so XFUZZ=1 FIRRTL_COVER=toggle EMU_TRACE=1 -j16
```

### 直接编译

```bash
export NOOP_HOME=$(pwd) 
make clean && make emu
```

### 简单的跑指令调试

```bash
./build/emu -i test.bin > test.log
# 带difftest
./build/emu -i ./ready-to-run/microbench.bin
# 不带difftest
./build/emu -i ./ready-to-run/microbench.bin --no-diff

./build/emu -i tmp/switch_mode_toU.bin -I 100 -C 300

./build/emu -i tmp/switch_mode_toU.bin -I 100 -C 500 -b 0 --dump-wave-full

./build/emu -i tmp/switch_mode_toU.bin -I 200 -C 500 -b 0 --dump-wave-full > tmp/test.log
```

```bash
# xfuzz
./ccover/xfuzz --coverage firrtl.toggle --max-runs 1000000 --fuzzing -o ./tmp/log --steps run --corpus-input ./rocket-riscvdv --elf /path/to/design/build/fuzzer -j8 -- --max-cycles 10000 --seed 2023

./ccover/xfuzz --coverage firrtl.toggle --max-runs 100000000 --fuzzing -o ./tmp/toggle --steps run --continue-on-errors --corpus-input ./corpus/linearized/riscv-dv --elf ./build/fuzzer -j1 -- --max-cycles 10000 --seed 2023
./ccover/xfuzz --coverage firrtl.line --max-runs 100000000 --fuzzing -o ./tmp/line --steps run --continue-on-errors --corpus-input ./corpus/linearized/riscv-dv --elf ./build/fuzzer -j1 -- --max-cycles 10000 --seed 2023

./build/fuzzer -f --continue-on-errors --corpus-input ./corpus/footprints/riscv-dv -c firrtl.toggle -- -I 1000 -C 10000 --as-footprint >tmp/toggle/pathfuzz.log 2> tmp/toggle/pathfuzz_stderr.log
./build/fuzzer -f --continue-on-errors --corpus-input ./corpus/footprints/riscv-dv -c firrtl.line -- -I 1000 -C 10000 --as-footprint >tmp/line/pathfuzz.log 2> tmp/line/pathfuzz_stderr.log
```

```vim
set encoding=utf-8 termencoding=utf-8 fileencoding=utf-8
```

```bash
ps -ef|grep oss-cad-suite|grep -v grep|cut -c 9-15
export https_proxy=http://192.168.9.31:7897 http_proxy=http://192.168.9.31:7897 all_proxy=socks5://192.168.9.31:7897
Panda4
docker exec -it ca0bdc8120b9 bin/bash
Panda2
docker exec -it 350a4a3c1e49 bin/bash
```