#!/bin/sh

while : ; do
	sh -c 'rosservice call /lk_flow/initialize_points'
done 1>/dev/null 2>&1
