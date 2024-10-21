# XFUZZ Environment Variables
export NOOP_HOME=$(pwd)
export XFUZZ_HOME=$(pwd)/ccover

# OSS CAD Suite
export OSS_CAD_SUITE_HOME=$(pwd)/../oss-cad-suite/environment

# Formal Tool CORPUS
export CORPUS_DIR=$(pwd)/ccover/Formal/coverTasks/hexbin

# Fuzz and Formal Tool Cover Points
export COVER_POINTS_OUT=$(pwd)/ccover/Formal/coverTasks

# Fuzz path
export FUZZ_PATH=$(pwd)/build/fuzzer

# RTL Source and Destination
# export RTL_SRC_DIR=$(pwd)/build/rtl
export RTL_SRC_DIR=$(pwd)/ccover/Formal/demo/rtl
export RTL_DST_DIR=$(pwd)/ccover/Formal/coverTasks/rtl

# sby template
export SBY_TEMPLATE=$(pwd)/ccover/Formal/template.sby