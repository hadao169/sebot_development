cd ~/ros2_sebot_ws/arduino/motorcontroller

# Compile
arduino-cli compile --fqbn arduino:avr:micro .

# Upload
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:micro .
