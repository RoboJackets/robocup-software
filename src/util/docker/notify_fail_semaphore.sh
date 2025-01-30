#!/bin/bash

set -e

if [ "$BRANCH_NAME" = "master" ]; then
    MESSAGE="<https://github.com/RoboJackets/robocup-software|$SEMAPHORE_REPO_SLUG> - Build #$SEMAPHORE_BUILD_NUMBER Failed on Semaphore CI!"
    curl -X POST --data-urlencode 'payload={"channel": "#robocup-status", "username": "Cthulhu", "text": "'"$MESSAGE"'", "icon_emoji": ":ghost:"}' $SLACK_WEBHOOK
fi
false
