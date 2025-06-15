#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Float64
import threading
import time

class TurtleBotMonitor:
    def __init__(self):
        """Initialize the TurtleBot3 monitoring system."""
        rospy.init_node('turtlebot_monitor', anonymous=True)
        
        # Initialize variables
        self.battery_level = 0.0
        self.battery_voltage = 0.0
        self.temperature = 0.0
        self.system_status = 'OK'
        
        # Create publishers for alerts
        self.alert_pub = rospy.Publisher('/turtlebot_alerts', DiagnosticArray, queue_size=10)
        
        # Subscribe to battery and sensor topics
        rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.diagnostics_callback)
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_system)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
    def battery_callback(self, data):
        """Callback for battery state updates."""
        self.battery_level = data.percentage
        self.battery_voltage = data.voltage
        
    def diagnostics_callback(self, data):
        """Callback for system diagnostics."""
        for status in data.status:
            if status.level >= DiagnosticStatus.WARN:
                self.system_status = 'WARNING'
                break
            elif status.level >= DiagnosticStatus.ERROR:
                self.system_status = 'ERROR'
                break
            else:
                self.system_status = 'OK'
        
    def check_battery(self):
        """Check battery levels and generate alerts if needed."""
        alert = DiagnosticStatus()
        alert.name = "Battery Status"
        
        if self.battery_level <= 20.0:
            alert.level = DiagnosticStatus.ERROR
            alert.message = "Battery critical: {}%".format(self.battery_level)
        elif self.battery_level <= 30.0:
            alert.level = DiagnosticStatus.WARN
            alert.message = "Battery low: {}%".format(self.battery_level)
        else:
            alert.level = DiagnosticStatus.OK
            alert.message = "Battery OK: {}%".format(self.battery_level)
            
        return alert
        
    def check_system_health(self):
        """Check overall system health."""
        alert = DiagnosticStatus()
        alert.name = "System Health"
        
        if self.system_status == 'ERROR':
            alert.level = DiagnosticStatus.ERROR
            alert.message = "System Error Detected"
        elif self.system_status == 'WARNING':
            alert.level = DiagnosticStatus.WARN
            alert.message = "System Warning"
        else:
            alert.level = DiagnosticStatus.OK
            alert.message = "System OK"
            
        return alert
        
    def monitor_system(self):
        """Main monitoring loop."""
        rate = rospy.Rate(1)  # 1Hz monitoring
        while not rospy.is_shutdown():
            # Create diagnostic array
            diag_array = DiagnosticArray()
            diag_array.header.stamp = rospy.Time.now()
            
            # Add battery status
            diag_array.status.append(self.check_battery())
            # Add system health status
            diag_array.status.append(self.check_system_health())
            
            # Publish alerts
            self.alert_pub.publish(diag_array)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = TurtleBotMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
