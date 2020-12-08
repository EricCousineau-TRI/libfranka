#!/bin/bash

if [[ $0 == "-bash" ]]; then
    echo "ERROR: Run as script"
    return 1
fi
set -eux

script_dir=$(cd $(dirname $0) && pwd)

if [[ ! -d ${script_dir}/venv ]]; then
(
    cd ${script_dir}
    python3 -m venv ./venv
    ./venv/bin/pip install -U wheel pip
    ./venv/bin/pip install -r ./requirements.txt
)
fi

set +eux
source ${script_dir}/venv/bin/activate
set -x
exec $@
