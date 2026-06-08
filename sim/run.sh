#!/bin/sh
# Compile and simulate the BFS engine with Icarus Verilog.
set -e
cd "$(dirname "$0")/.."

mkdir -p sim

iverilog -g2012 -Wall \
    -o sim/bfs_sim \
    rtl/vertex_fifo.sv \
    rtl/visited_bitmap.sv \
    rtl/axi4_rd_master.sv \
    rtl/bfs_ctrl.sv \
    rtl/bfs_top.sv \
    tb/hbm_mem_model.sv \
    tb/bfs_tb.sv

echo "Compiled OK — running simulation..."
vvp sim/bfs_sim
