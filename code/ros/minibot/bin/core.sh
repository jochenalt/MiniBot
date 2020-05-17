roscore &

# before we can call  rosparam, roscore neds to be up
sleep 5
rosparam set mongodb_port 62345
rosparam set mongodb_host localhost

# start mongo db
rosrun mongodb_store mongodb_server.py &
