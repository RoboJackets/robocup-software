#!/bin/bash

set -e

if [ "$CIRCLE_BRANCH" = "master" ]; then
    MESSAGE="<$CIRCLE_REPOSITORY_URL|$CIRCLE_PROJECT_REPONAME> - Build #$CIRCLE_BUILD_NUM Failed on CircleCI! \n<$CIRCLE_BUILD_URL|Link to Failure>"
    curl -X POST --data-urlencode 'payload={"channel": "#robocup-status", "username": "Cthulhu", "text": "'"$MESSAGE"'", "icon_emoji": ":ghost:"}' $SLACK_WEBHOOK
fi
false
