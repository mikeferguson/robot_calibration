#!/bin/bash
# Export CI environment info
ci_env=`bash <(curl -s https://codecov.io/env)`
export DOCKER_RUN_OPTS="$DOCKER_RUN_OPTS $ci_env"
