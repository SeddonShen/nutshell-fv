git apply ./patches/E1_SLTI.patch
mill chiselModule.test.testOnly formal.NutCoreFormalSpec
mv ./test_run_dir/Elaborate_chirvformal_SystemVerilog/NutCore.sv ./verification_files/NutCore_E1.sv
git apply -R ./patches/E1_SLTI.patch
git apply ./patches/E2_SUB.patch
mill chiselModule.test.testOnly formal.NutCoreFormalSpec
mv ./test_run_dir/Elaborate_chirvformal_SystemVerilog/NutCore.sv ./verification_files/NutCore_E2.sv
git apply -R ./patches/E2_SUB.patch
git apply ./patches/E3_BNE.patch
mill chiselModule.test.testOnly formal.NutCoreFormalSpec
mv ./test_run_dir/Elaborate_chirvformal_SystemVerilog/NutCore.sv ./verification_files/NutCore_E3.sv
git apply -R ./patches/E3_BNE.patch
git apply ./patches/E4_BLTU.patch
mill chiselModule.test.testOnly formal.NutCoreFormalSpec
mv ./test_run_dir/Elaborate_chirvformal_SystemVerilog/NutCore.sv ./verification_files/NutCore_E4.sv
git apply -R ./patches/E4_BLTU.patch
git apply ./patches/E5_ADDI.patch
mill chiselModule.test.testOnly formal.NutCoreFormalSpec
mv ./test_run_dir/Elaborate_chirvformal_SystemVerilog/NutCore.sv ./verification_files/NutCore_E5.sv
git apply -R ./patches/E5_ADDI.patch