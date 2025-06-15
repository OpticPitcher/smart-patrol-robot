// Import
const fs = require("fs")
const path = require('path');
const express = require('express')
const app = express()
const bcrypt = require('bcrypt')
const passport = require('passport')
const flash = require('express-flash')
const session = require('express-session')
const methodOverride = require('method-override')
const WebSocket = require('ws')
const rclnodejs = require('rclnodejs');

// Set ROS_DOMAIN_ID
process.env.ROS_DOMAIN_ID = '30';

// Constants for Waffle Pi
const MAX_LINEAR_VEL = 0.26;
const MAX_ANGULAR_VEL = 1.82;
const LINEAR_STEP = 0.01;
const ANGULAR_STEP = 0.1;

// Passport Initialize
const initializePassport = require('./passport-config');
initializePassport(
  passport,
  username => users.users.find(user => user.username === username),
  id => users.users.find(user => user.id === id)
)

// Database Configurations
const users = require(__dirname + '/db.json')
userspath = __dirname + '/db.json'

// Express Configurations
app.set('view-engine', 'ejs')
app.use(express.urlencoded({ extended: false }))
app.use(flash())
app.use(express.static('./views'));
app.use(session({ secret: "water", resave: false, saveUninitialized: false }))
app.use(passport.initialize())
app.use(passport.session())
app.use(methodOverride('_method'))

// Initialize ROS 2 node
let node;
let publisher;
let cameraSubscriber;
let scanSubscriber;
let mapSubscriber;
let batterySubscriber;
let lastBatteryUpdate = 0;
const BATTERY_UPDATE_INTERVAL = 60000; // 60 seconds in milliseconds

// Store current velocities
let currentLinearVel = 0.0;
let currentAngularVel = 0.0;

async function initROS() {
  try {
    await rclnodejs.init();
    node = new rclnodejs.Node('web_controller');
    publisher = node.createPublisher('geometry_msgs/msg/Twist', '/cmd_vel');
    
    // Subscribe to battery state
    batterySubscriber = node.createSubscription(
      'sensor_msgs/msg/BatteryState',
      '/battery_state',
      (msg) => {
        try {
          const now = Date.now();
          // Only send update if enough time has passed
          if (now - lastBatteryUpdate >= BATTERY_UPDATE_INTERVAL) {
            lastBatteryUpdate = now;
            wss.clients.forEach((client) => {
              if (client.readyState === WebSocket.OPEN) {
                client.send(JSON.stringify({
                  type: 'battery',
                  data: {
                    percentage: msg.percentage,
                    voltage: msg.voltage,
                    current: msg.current,
                    charge: msg.charge,
                    capacity: msg.capacity,
                    design_capacity: msg.design_capacity,
                    power_supply_status: msg.power_supply_status,
                    power_supply_health: msg.power_supply_health,
                    power_supply_technology: msg.power_supply_technology,
                    present: msg.present
                  }
                }));
              }
            });
          }
        } catch (error) {
          console.error('Error processing battery data:', error);
        }
      }
    );

    // Subscribe to camera feed
    cameraSubscriber = node.createSubscription(
      'sensor_msgs/msg/CompressedImage',
      '/camera/image_raw/compressed',
      (msg) => {
        try {
          const imageData = Buffer.from(msg.data).toString('base64');
          wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
              client.send(JSON.stringify({
                type: 'camera',
                data: imageData
              }));
            }
          });
        } catch (error) {
          console.error('Error processing camera data:', error);
        }
      }
    );

    // Subscribe to LiDAR scan data
    scanSubscriber = node.createSubscription(
      'sensor_msgs/msg/LaserScan',
      '/scan',
      (msg) => {
        try {
          wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
              client.send(JSON.stringify({
                type: 'scan',
                data: {
                  ranges: msg.ranges,
                  angle_min: msg.angle_min,
                  angle_max: msg.angle_max,
                  angle_increment: msg.angle_increment,
                  range_min: msg.range_min,
                  range_max: msg.range_max
                }
              }));
            }
          });
        } catch (error) {
          console.error('Error processing scan data:', error);
        }
      }
    );

    // Subscribe to map data
    mapSubscriber = node.createSubscription(
      'nav_msgs/msg/OccupancyGrid',
      '/map',
      (msg) => {
        try {
          const mapData = {
            width: msg.info.width,
            height: msg.info.height,
            resolution: msg.info.resolution,
            origin: {
              x: msg.info.origin.position.x,
              y: msg.info.origin.position.y
            },
            data: msg.data
          };
          
          wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
              client.send(JSON.stringify({
                type: 'map',
                data: mapData
              }));
            }
          });
        } catch (error) {
          console.error('Error processing map data:', error);
        }
      }
    );

    // Publish velocity command
    function publishVelocity() {
      const twist = {
        linear: { x: currentLinearVel, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: currentAngularVel }
      };
      publisher.publish(twist);
    }

    // WebSocket server setup
    const wss = new WebSocket.Server({ port: 8766 });

    wss.on('connection', function connection(ws) {
      console.log('New WebSocket connection');
      
      ws.on('message', function incoming(message) {
        try {
          const data = JSON.parse(message);
          
          if (data.type === 'control') {
            const command = data.command;
            
            switch(command) {
              case 'forward':
                currentLinearVel = Math.min(currentLinearVel + LINEAR_STEP, MAX_LINEAR_VEL);
                break;
              case 'backward':
                currentLinearVel = Math.max(currentLinearVel - LINEAR_STEP, -MAX_LINEAR_VEL);
                break;
              case 'left':
                currentAngularVel = Math.min(currentAngularVel + ANGULAR_STEP, MAX_ANGULAR_VEL);
                break;
              case 'right':
                currentAngularVel = Math.max(currentAngularVel - ANGULAR_STEP, -MAX_ANGULAR_VEL);
                break;
              case 'stop':
                currentLinearVel = 0.0;
                currentAngularVel = 0.0;
                break;
              case 'full_forward':
                currentLinearVel = MAX_LINEAR_VEL;
                currentAngularVel = 0.0;
                break;
              case 'battery':
                // Force battery update by resetting the timer
                lastBatteryUpdate = 0;
                break;
            }
            
            publishVelocity();
            
            // Send response back to client
            ws.send(JSON.stringify({
              status: 'success',
              command: command,
              velocities: {
                linear: currentLinearVel,
                angular: currentAngularVel
              }
            }));
          } else if (data.type === 'mode') {
            if (data.mode === 'auto') {
              // Stop the robot when switching to auto mode
              currentLinearVel = 0.0;
              currentAngularVel = 0.0;
              publishVelocity();
            }
          }
        } catch (error) {
          console.error('Error processing message:', error);
          ws.send(JSON.stringify({
            status: 'error',
            message: error.message
          }));
        }
      });
    });

    node.spin();
    console.log('ROS 2 node initialized successfully on domain ID:', process.env.ROS_DOMAIN_ID);
  } catch (error) {
    console.error('Error initializing ROS 2 node:', error);
  }
}

//--------------------------------------GETS--------------------------------------//
app.get('/', checkAuthenticated, (req, res) => {
  res.render('index.ejs', { username: req.user.username })
})

app.get('/login', checkNotAuthenticated, (req, res) => {
  res.render('login.ejs')
})

app.get('/register', checkNotAuthenticated, (req, res) => {
  res.render('register.ejs')
})

app.get('/style.css', (req, res) => {
  res.sendFile(__dirname + '/views/style.css')
});

app.get('/minimap.jpg', (req, res) => {
  res.sendFile(__dirname + '/views/minimap.webp')
});

app.get('/404', (req, res) => {
  username = ""
  filenow = req.query
  res.render('404.ejs', { filename: filenow, username: username })
  // res.send(`<?xml version='1.0' encoding='UTF-8'?><Error><Code>404</Code><Message>Not Found</Message><Details>The requested asset was not found inside the ${req.user.username} Cloud Storage bucket.</Details></Error>`)
});

app.get('/*', auth404, (req, res) => {
  username = req.user.username
  filenow = req.query
  res.render('404.ejs', { filename: filenow, username: username })
});
//--------------------------------------POST--------------------------------------//
app.post('/login', checkNotAuthenticated, passport.authenticate('local', {
  successRedirect: '/',
  failureRedirect: '/login',
  failureFlash: true
}))

app.post('/register', checkNotAuthenticated, async (req, res) => {
  try {
    // Check if username already exists
    const existingUser = users.users.find(user => user.username === req.body.username)
    if (existingUser) {
      req.flash('error', 'Username already exists')
      return res.redirect('/register')
    }

    // Hash the password
    const hashedPassword = await bcrypt.hash(req.body.password, 10)
    // Make an object to push the user information
    register = {
      id: Date.now().toString(),
      username: req.body.username,
      fullname: req.body.fullname,
      password: hashedPassword
    }
    // Push the registration info to db
    users.users.push(register)
    // Add indentations to make the db readable
    fs.writeFileSync(userspath, JSON.stringify(users, null, 4));
    res.redirect('/login')
  } catch (e) {
    console.log(e)
    req.flash('error', 'Error during registration')
    res.redirect('/register')
  }
})

//-----------------------------DELETE Route for Logout----------------------------//
app.delete('/logout', (req, res, next) => {
  req.logOut(function (err) { return next(err); })
  res.redirect('/login')
})
//--------------------------------User Not Signed In------------------------------//
function checkAuthenticated(req, res, next) {
  if (req.isAuthenticated()) { return next() }
  res.redirect('/login')
}
//---------------------------------User IS Signed In------------------------------//
function checkNotAuthenticated(req, res, next) {
  if (req.isAuthenticated()) { return res.redirect('/') }
  next()
}
//--------------------------------User Not Signed In------------------------------//
function auth404(req, res, next) {
  if (req.isAuthenticated()) { return next() }
  res.redirect('/404')
}
//-------------------------------Server Startup Config----------------------------//
var server = app.listen(5002, function () {
  var port = server.address().port
  var family = server.address().family
  var address = server.address().address
  if (address == "::") { address = "this ratio mf" }
  console.log("Server running on Port: http://localhost:" + port, "| Family:", family, "| Address", address)
});

// Initialize ROS 2 before starting the server
initROS().then(() => {
  // ... rest of your existing code ...
});
