import subprocess
import re
import time

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

if __name__ == "__main__":
    ip = "10.20.0.30"  # Replace with the IP address you want to ping
    
    rate = 0.1  # 10Hz -> 0.1 seconds interval
    
    while True:
        avg_rtt, packet_loss = ping_host(ip)
        
        if avg_rtt is not None and packet_loss is not None:
            connection = calculate_connection_quality(avg_rtt, packet_loss)
            print(f"Connection Quality: {connection}%")
            print(f"Average RTT: {avg_rtt} ms")
            print(f"Packet Loss: {packet_loss}%")
        else:
            print("Failed to retrieve ping statistics.")
        
        time.sleep(rate)