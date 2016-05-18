#!/bin/bash

set -e

MBED_INDEX=0 make robot2015-test-radio-receiver-prog
MBED_INDEX=1 make robot2015-test-radio-sender-prog
