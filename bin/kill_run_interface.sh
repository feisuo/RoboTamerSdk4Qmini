#!/bin/bash

echo ">>> ps aux | grep run_interface"
ps aux | grep -E '(^|/)run_interface( |$)' | grep -v grep || true

pids=$(pgrep -x run_interface || true)

if [ -z "$pids" ]; then
  echo "No run_interface process found."
  exit 0
fi

echo ">>> kill -9 $pids"
kill -9 $pids
echo "Done."

