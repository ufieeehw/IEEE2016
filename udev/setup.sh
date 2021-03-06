#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cp $DIR/98-ftdi.rules /etc/udev/rules.d/98-ftdi.rules
cp $DIR/47-hokuyo.rules /etc/udev/rules.d/47-hokuyo.rules
cp $DIR/99-cameras.rules /etc/udev/rules.d/99-cameras.rules
udevadm control --reload-rules
udevadm trigger
