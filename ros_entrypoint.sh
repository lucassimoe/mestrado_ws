#!/bin/bash
set -e

# setup ros environment
source $MESTRADO_WS/devel/setup.bash
exec "$@"