#!/usr/bin/env bash
set -ue

SRC_DIR=$(cd $(dirname ${BASH_SOURCE:-$0})/../; pwd)

cd $SRC_DIR
docker build -t rt-net/thouzer_driver -f docker/Dockerfile .