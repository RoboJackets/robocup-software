nodes=$(ros2 node list)
for node in ${nodes}
do
    ros2 param dump ${node} --print
done
