# Build example using the .c files in this directory and the static libartl.a
odom_adapt: odom_adapt.cpp
	g++ $^ -o $@ -I/opt/ros/noetic/include -L/opt/ros/noetic/lib -Wl,-rpath,opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -ltf2_ros -ltf2 -ltf_conversions -pthread -lactionlib

# Clean outputs from both this directory and libartl
clean:
	rm -f odom_adapt

.PHONY: clean
