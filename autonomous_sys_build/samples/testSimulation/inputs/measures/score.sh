#!/usr/bin/env bash
cat /simulation/testSimulation/outputs/measure.csv | awk -F',' '{sum+=$2; ++n} END { print sum/n }' > /simulation/testSimulation/outputs/score.out
cat /simulation/testSimulation/outputs/score.out