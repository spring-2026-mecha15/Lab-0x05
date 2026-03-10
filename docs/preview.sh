#!/bin/bash
cd "$(dirname "$0")"
source ../.venv/bin/activate
make html && open -a Safari build/html/index.html
