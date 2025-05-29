# NutShell Formal Verification

This project is a case study using
[riscv-spec-core](https://github.com/iscas-tis/riscv-spec-core) on Nutshell for
formal verification.

## NutShell (果壳)

[NutShell](https://github.com/OSCPU/NutShell) is a processor developed by the
OSCPU (Open Source Chip Project by University) team.
Currently, it supports riscv64/32.
More information about NutShell see its
[GitHub repo](https://github.com/OSCPU/NutShell).

## Run Verification Directly

Clone submodule:

```shell
git submodule update --init --recursive
```

Run verification:

```shell
mill chiselModule.test.testOnly formal.NutCoreFormalSpec
```

This will run the test case `formal.NutCoreFormalSpec`, which transforms NutCore
(core computing unit) with assertions and `SpecCore` in riscv-spec-core to a
transaction system and then passes it to the formal verification backend.

## Modifications On NutShell

Search `Formal` in source code to see the main modifications.

## JSA25
这里是JSA25投稿重新进行chirvformal和riscv-formal效率对比实验的仓库，主要修改：
- 修改工具链生成迁移系统的方式
- 修改到最新版的参考模型

### 使用方法
生成SystemVerilog文件：
```bash
mill chiselModule.test.testOnly formal.NutCoreFormalSpec
# 会在test_run_dir/Elaborate_chirvformal_SystemVerilog/  生成NutCore.sv文件
# 将生成的sv文件复制到verification_files/NutCore.sv
cd ./verification_files
sby SimTop.sby -f
```
```