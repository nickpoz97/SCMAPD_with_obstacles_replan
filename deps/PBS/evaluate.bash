#!/bin/bash

cd $(realpath "$(dirname "${BASH_SOURCE:-$0}")")
PATHS_DIR="./paths"

mkdir -p $PATHS_DIR

for N_AGENTS in $(seq "$1" "$2" "$3")
do
    for N_TASKS in $(seq "$4" "$5" "$6")
    do
        SUBFOLDER_ID=a"$N_AGENTS"_t"$N_TASKS"
        SUBFOLDER_METHOD="ta_ortools"
        mkdir -p $PATHS_DIR/$SUBFOLDER_ID/$SUBFOLDER_METHOD

        for i in $(seq 0 $((N_INSTANCES-1)))
        do
          ASSIGNMENT=instances/"$SUBFOLDER_ID"/ta_ortools/"$i".assignment
          OUTPUT_PATHS=$PATHS_DIR/$SUBFOLDER_ID/$SUBFOLDER_METHOD/paths_$i.txt
          [[ -f $ASSIGNMENT ]] && $PBS_EXE_PATH -m env/grid.map -a "$ASSIGNMENT" -k "$N_AGENTS" -t 60 --outputPaths="$OUTPUT_PATHS"
        done
    done
done