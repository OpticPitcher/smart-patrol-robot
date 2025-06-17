# Smart Patrol Robot

A web-based control system for TurtleBot3 robots, providing real-time monitoring and control capabilities for security and patrol applications.

## Features

- Web-based control interface
- Real-time robot monitoring
- Secure authentication system
- ROS 2 integration
- Battery and system diagnostics
- Emergency stop functionality
- Patrol route management
- Environmental monitoring

## System Requirements

### Hardware
- TurtleBot3 (Burger or Waffle Pi)
- Compatible LiPo battery
- Control station (computer)
- Network connection

### Software
- ROS 2 Foxy Fitzroy
- Node.js 16.x or later
- Python 3.8 or later
- Ubuntu 20.04 LTS (recommended)

## Installation

### 1. System Setup
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y build-essential python3-pip python3-rosdep
```

### 2. ROS 2 Installation
```bash
# Setup ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Foxy
sudo apt update
sudo apt install -y ros-foxy-desktop
```

### 3. Node.js Installation
```bash
# Install Node.js
sudo apt install -y nodejs npm

# Verify installation
node --version
npm --version
```

### 4. Project Setup
```bash
# Clone repository
git clone https://github.com/OpticPitcher/smart-patrol-robot.git
cd smart-patrol-robot

# Install dependencies
npm install

# Create database file
touch db.json
```

### 5. Configure Environment
```bash
# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=30

# Add to bashrc
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
```

## Running the System

### Start ROS 2 Node
```bash
source /opt/ros/foxy/setup.bash
node server.js
```

### Start Web Server
```bash
npm start
```

### Access Web Interface
Open web browser and navigate to: http://localhost:5002

## Usage

1. **Login**
   - Default admin credentials (change after first login)
   - Username: admin
   - Password: admin

2. **Basic Controls**
   - Use arrow keys or joystick for movement
   - Adjust speed with slider
   - Emergency stop button for immediate halt

3. **Monitoring**
   - View battery level and voltage
   - Check system diagnostics
   - Monitor temperature
   - View alerts and warnings

## Security

- All user data is encrypted
- Secure authentication system
- Role-based access control
- Regular security updates

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request

## License

ISC License

## Support

For support, please open an issue in the GitHub repository.

## Acknowledgments

- ROS 2
- TurtleBot3
- Node.js
- Express.js
- Passport.js
