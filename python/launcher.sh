#!/bin/sh
while true; do
	python /home/bugs/SensorReader.py &
	wait $!
	sleep 10
done
exit


