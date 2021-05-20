#!/bin/bash

# Signa Handler
trap 'echo "Programm mit <Strg>+<c> (SIGINT) beendet"; exit' 2

# Nach kurzer Zeit, wenn Luefter aktiv, "friert" System ein
# sudo sh -c "echo 255 > /sys/devices/pwm-fan/target_pwm"

./detector/detector | ./filter_1/transition

# can_dispatcher muss noch implementiert werden
# ./detector/detector | ./filter_1/transition | ./can/dispatcher
