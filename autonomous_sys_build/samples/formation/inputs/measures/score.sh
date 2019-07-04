#!/usr/bin/env bash
cat /simulation/formation/outputs/measure.csv | awk -F',' '{sum+=$2; ++n} END { print sum/n }' > /simulation/formation/outputs/score.out
cat /simulation/formation/outputs/score.out