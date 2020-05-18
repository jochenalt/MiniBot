#!/bin/bash

rosservice find /rosout 2> /dev/null
status=$?
if [ $status -eq 0 ]; then
   echo "roscore is running already"
else
   echo "starting roscore"
   roscore -w 16 &
   sleep 1
   rosservice find /rosout
   while [ $? -ne 0 ]
   do
      echo "not yet up"
      sleep 1
      rosservice find /rosout
   done
   echo "roscore up and running"
fi

echo "setting localhost:62345 for mongodb"
rosparam set mongodb_port 62345
rosparam set mongodb_host localhost

# start mongo db
echo "starting mongodb"
rosrun mongodb_store mongodb_server.py &
echo "done"

sleep 5
