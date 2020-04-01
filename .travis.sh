#!/bin/bash

# Setup environment correctly so that CI actually runs
ci_env=`bash <(curl -s https://codecov.io/env)`
DOCKER_RUN_OPTS="$DOCKER_RUN_OPTS $ci_env"

echo "DOCKER_RUN_OPTS=$DOCKER_RUN_OPTS"

# http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Call into industrial CI
source /root/src/.industrial_ci/.travis.sh
