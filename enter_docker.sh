#!/bin/bash

# 容器名称或ID
CONTAINER_NAME_OR_ID="lm-calibr-humble-container"

# 检查容器是否正在运行
if docker ps | grep -q $CONTAINER_NAME_OR_ID; then
    # 进入容器的终端
    docker exec -it $CONTAINER_NAME_OR_ID /bin/bash
else
    echo "docker $CONTAINER_NAME_OR_ID is not running"
fi