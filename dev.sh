#!/bin/bash
# Convenience script for local builds - feel free to change it to your liking

set -eu

mkdir -p build 

# -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -Wall -Wextra -Wno-missing-braces -pedantic -fprofile-instr-generate -fcoverage-mapping" 


HI='\033[0;32m'
NC='\033[0m'

cmake -S . \
			-B build \
			-G Ninja \
			-DCMAKE_CXX_COMPILER=clang++ \
			-DCMAKE_C_COMPILER=clang \
      -DTYPES="FAST_FIXED(32,16),FAST_FIXED(64,8)" \
      -DSIZES="S(291,84),S(36,84)" \
      -DCMAKE_CXX_FLAGS="" \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=1 
cmake --build build --parallel $(nproc)
cp -f --remove-destination build/compile_commands.json .

if [ $# -ne 0 ] ; then
	echo -e "$HI--------------------------------------$NC\n"
  if [ "$1" == "run" ]; then
    set -eux
    ./build/dyn-types --p-type='FAST_FIXED(64,8)' --v-type='FAST_FIXED(64,8)' --v-flow-type='FAST_FIXED(64,8)' -j7 field_giant.txt
  elif [ "$1" == "test" ]; then
    pushd build/tests > /dev/null
    rm -rf coverage
    mkdir -p coverage/report
    test_name=specify_test_executable_in_dev_sh
    LLVM_PROFILE_FILE="coverage/%p.profraw" ./$test_name
    llvm-profdata merge -sparse coverage/*.profraw -o coverage/$test_name.profdata
    llvm-cov show ./$test_name -instr-profile=coverage/$test_name.profdata -format=html -output-dir=coverage/report --ignore-filename-regex=build/.*
    popd > /dev/null
  fi
fi
