#!/usr/bin/env python3
import subprocess
import re
import requests

import rospy
from std_msgs.msg import Float32

def get_public_ip():
    try:
        response = requests.get('https://api.ipify.org?format=json')
        response.raise_for_status()  # Raise an exception for HTTP errors
        ip_info = response.json()
        return ip_info['ip']
    except requests.RequestException as e:
        print(f"Error occurred: {e}")
        return None
        
def ping_host(ip, count=1):
        try:
            # Run the ping command
            output = subprocess.check_output(['ping', '-c', str(count), ip], universal_newlines=True)
            
            # Parse the output
            # Extract the average round-trip time (RTT)
            rtt_match = re.search(r'rtt min/avg/max/mdev = .*?/([0-9.]+)/', output)
            avg_rtt = float(rtt_match.group(1)) if rtt_match else None
            
            # Extract packet loss percentage
            loss_match = re.search(r'(\d+)% packet loss', output)
            packet_loss = int(loss_match.group(1)) if loss_match else None
            
            return avg_rtt, packet_loss

        except subprocess.CalledProcessError as e:
            print("Failed to ping host:", e)
            return None, None
    
def calculate_connection_quality(avg_rtt, packet_loss):
    # Initial connection quality percentage
    quality = 100

    # Decrease quality based on packet loss
    if packet_loss:
        quality -= packet_loss * 1.5  # Packet loss has a significant impact

    # Decrease quality based on RTT
    if avg_rtt:
        if avg_rtt < 100:
            quality -= avg_rtt * 0.1  # Minor impact for low RTT
        elif avg_rtt < 300:
            quality -= avg_rtt * 0.2  # Moderate impact for medium RTT
        else:
            quality -= avg_rtt * 0.5  # High impact for high RTT

    # Ensure quality is within 0-100 range
    quality = max(0, min(100, quality))

    return quality

def getConnectionQuality(ip):
    avg_rtt, packet_loss = ping_host(ip)
    if avg_rtt is not None and packet_loss is not None:
        connection = calculate_connection_quality(avg_rtt, packet_loss)
        return connection
    return 0
    
if __name__ == "__main__":
    try:
        rospy.init_node('robot_connection', anonymous=True)
        connection_pub = rospy.Publisher('robot_connection', Float32, queue_size=1, latch=True)
        
        # ip = get_public_ip()
        ip = "94.206.14.42"
        connection = Float32()
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            
            connection.data = getConnectionQuality(ip)
            connection_pub.publish(connection)
            print(f"Connection quality: {connection}")
            r.sleep()
    except rospy.ROSInterruptException:
        pass
