#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Step 1: Define variables
VENV_DIR="venv"           # Directory for the virtual environment
REQUIREMENTS_FILE="requirements.txt"  # Path to requirements.txt (in root)
MAIN_SCRIPT="src/main.py"  # Path to main.py (in src/)

# Step 2: Check for Python3
if ! command -v python3 &> /dev/null
then
    echo "Python3 is not installed. Please install Python3 and try again."
    exit 1
fi

# Step 3: Create a virtual environment
echo "Creating virtual environment in $VENV_DIR..."
python3 -m venv $VENV_DIR

# Step 4: Activate the virtual environment
echo "Activating virtual environment..."
source $VENV_DIR/bin/activate

# Step 5: Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Step 6: Install dependencies
if [[ -f "$REQUIREMENTS_FILE" ]]; then
    echo "Installing dependencies from $REQUIREMENTS_FILE..."
    pip install -r $REQUIREMENTS_FILE
else
    echo "$REQUIREMENTS_FILE not found. Skipping dependency installation."
fi

# Step 7: Run the main Python script
if [[ -f "$MAIN_SCRIPT" ]]; then
    echo "Running $MAIN_SCRIPT..."
    python $MAIN_SCRIPT
else
    echo "$MAIN_SCRIPT not found. Exiting."
    exit 1
fi

# Optional: Deactivate virtual environment
echo "Deactivating virtual environment..."
deactivate
