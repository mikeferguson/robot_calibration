#!/bin/bash

# Setup environment correctly so that CI actually runs
ci_env=`bash <(curl -s https://codecov.io/env)`
DOCKER_RUN_OPTS="$DOCKER_RUN_OPTS $ci_env"

echo "DOCKER_RUN_OPTS=$DOCKER_RUN_OPTS"

# Call into industrial CI
source .industrial/.travis.sh
