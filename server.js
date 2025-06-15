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

async function initROS() {
  try {
    await rclnodejs.init();
    node = new rclnodejs.Node('web_controller');
    publisher = node.createPublisher('geometry_msgs/msg/Twist', '/cmd_vel');
    
    // Subscribe to camera feed
    cameraSubscriber = node.createSubscription(
      'sensor_msgs/msg/CompressedImage',
      '/camera/image_raw/compressed',
      (msg) => {
        try {
          // Convert the image data to base64
          const imageData = Buffer.from(msg.data).toString('base64');
          
          // Broadcast the image data to all connected WebSocket clients
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

    node.spin();
    console.log('ROS 2 node initialized successfully on domain ID:', process.env.ROS_DOMAIN_ID);
  } catch (error) {
    console.error('Error initializing ROS 2 node:', error);
  }
}

// WebSocket server setup
const wss = new WebSocket.Server({ port: 8766 });

wss.on('connection', function connection(ws) {
  console.log('New WebSocket connection');
  
  ws.on('message', function incoming(message) {
    try {
      const data = JSON.parse(message);
      const command = data.command;
      
      // Create Twist message
      const twist = {
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
      };
      
      // Map commands to robot movements
      switch(command) {
        case 'forward':
          twist.linear.x = 0.5;
          twist.angular.z = 0.0;
          break;
        case 'backward':
          twist.linear.x = -0.5;
          twist.angular.z = 0.0;
          break;
        case 'left':
          twist.linear.x = 0.0;
          twist.angular.z = 0.5;
          break;
        case 'right':
          twist.linear.x = 0.0;
          twist.angular.z = -0.5;
          break;
        case 'stop':
          twist.linear.x = 0.0;
          twist.angular.z = 0.0;
          break;
      }
      
      // Publish the message
      publisher.publish(twist);
      
      // Send response back to client
      ws.send(JSON.stringify({
        status: 'success',
        command: command
      }));
    } catch (error) {
      console.error('Error processing message:', error);
      ws.send(JSON.stringify({
        status: 'error',
        message: error.message
      }));
    }
  });
});

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
