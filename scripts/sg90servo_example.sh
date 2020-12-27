#!/bin/bash

:<<'COPYRIGHT'
MIT License
Copyright (c) 2020 Pratik M Tambe <enthusiasticgeek@gmail.com>
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
COPYRIGHT


# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
        echo "** Trapped CTRL-C"
        exit 2
}


if (( $# < 1 )); then
    echo -n -e "Pass argument # 1 as number of iterations\n"
    exit 1
fi

ITERATIONS=${1}
COUNTER=0
while [  $COUNTER -lt ${ITERATIONS} ]; do
echo 10 > /sys/devices/virtual/AutomatorServo/Servo/op_iterations
echo 2000 > /sys/devices/virtual/AutomatorServo/Servo/period_on
echo 18000 > /sys/devices/virtual/AutomatorServo/Servo/period_off
echo 1 > /sys/devices/virtual/AutomatorServo/Servo/op_start
#sleep 1
#echo 1500 > /sys/devices/virtual/AutomatorServo/Servo/period_on
#echo 18500 > /sys/devices/virtual/AutomatorServo/Servo/period_off
#echo 1 > /sys/devices/virtual/AutomatorServo/Servo/op_start
usleep 1000
echo 1000 > /sys/devices/virtual/AutomatorServo/Servo/period_on
echo 19000 > /sys/devices/virtual/AutomatorServo/Servo/period_off
echo 1 > /sys/devices/virtual/AutomatorServo/Servo/op_start
let COUNTER=COUNTER+1 
usleep 1000
done
