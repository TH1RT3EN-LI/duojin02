#!/usr/bin/env bash
set -euo pipefail
report_dir="$(cd -- "$(dirname -- "$BASH_SOURCE")" && pwd)"
mkdir -p "$report_dir/build"
cd "$report_dir"
for report_pass in 1 2 3; do
  xelatex -interaction=nonstopmode -halt-on-error -output-directory=build main.tex > "build/pass-$report_pass.log" 2>&1
done
cp build/main.pdf ../duojin02_cuda_acceleration_reconstruction_report.pdf
