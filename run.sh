#!/bin/bash

echo "--- Setting up Project Dependencies ---"

# --- Part 1: Install Node.js Dependencies ---
echo "Checking for Node.js dependencies..."
npm install

# --- Part 2: Install Python Dependencies ---
echo "Checking for Python dependencies..."
pip install -r requirements.txt

echo ""
echo "--- Setup Complete ---"
echo ""

# --- Part 3: Dynamic Startup Menu ---
echo "What would you like to start?"
echo "  1) Start Node.js Web Server (server.js)"
echo "  2) Start Python TurtleBot Server (turtlebot_server.py)"
echo "  3) Start Python TurtleBot Wrapper (turtlebot_wrapper.py)"
read -p "Enter your choice (1, 2, or 3): " choice

case $choice in
    1)
        echo "Starting Node.js Web Server..."
        # This runs your main web application
        node server.js
        ;;
    2)
        echo "Starting Python TurtleBot Server..."
        # This runs the FastAPI/Python server
        python3 turtlebot_server.py
        ;;
    3)
        echo "Starting Python TurtleBot Wrapper..."
        # This runs the direct robot control script
        python3 turtlebot_wrapper.py
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac