#!/bin/bash

set -e

curl -X POST --data-urlencode 'payload={"channel": "#robocup-status", "username": "Cthulhu", "text": "The Build Failed!\n$CIRCLE_BUILD_URL", "icon_emoji": ":ghost:"}' $SLACK_WEBHOOK
false
