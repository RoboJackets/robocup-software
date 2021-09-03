nodes=$(ros2 node list)
for node in ${nodes}
do
    if [[ "$node" != "/rqt*" ]]; then
        ros2 param dump ${node} --print
    fi
done
