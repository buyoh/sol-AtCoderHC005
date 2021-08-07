#!/bin/bash

set -eu
cd `dirname $0`/..

pushd third_party/tools > /dev/null
echo $RANDOM > seeds.txt
cargo run --release --bin gen seeds.txt  > /dev/null
popd > /dev/null

cat third_party/tools/in/0000.txt
