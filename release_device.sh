#!/bin/sh

PID=`lsof | grep usbmodem | awk '{print $2}'`
kill -9 $PID
