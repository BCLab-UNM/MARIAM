#!/bin/bash

# Directory containing udev rules
RULES_DIR="./rules"

# Check if the directory exists
if [ ! -d "$RULES_DIR" ]; then
  echo "Directory $RULES_DIR does not exist."
  exit 1
fi

# Copy udev rules to /etc/udev/rules.d/
for rule in "$RULES_DIR"/*.rules; 
do
  if [ -f "$rule" ]; then
    sudo cp "$rule" /etc/udev/rules.d/
    echo "Copied $rule to /etc/udev/rules.d/"
  else
    echo "No .rules files found in $RULES_DIR."
    exit 1
  fi
done

# Reload udev rules
sudo udevadm control --reload-rules
echo "Udev rules reloaded."

# Trigger the new rules
sudo udevadm trigger
echo "Udev rules applied."

echo "All udev rules in $RULES_DIR have been applied."
