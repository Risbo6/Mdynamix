from ast import While
import sys
import trio
import subprocess
import netifaces as ni
import time

from datetime import datetime

PORT = 12345
mdynamix_ip = '0'
server_ip='160.98.26.115'

async def sender(client_stream):
        global mdynamix_ip
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")

        msg = f"Mdynamix-{mdynamix_ip}-{current_time}"
        await client_stream.send_all(msg.encode("UTF-8"))



async def parent():
    client_stream = await trio.open_tcp_stream(server_ip, PORT)
    async with client_stream:
        async with trio.open_nursery() as nursery:
            nursery.start_soon(sender, client_stream)



# Ethernet
while True :
    try:
        _ = ni.ifaddresses('eth1')[ni.AF_INET][0]['addr']
        break
        

    except:
        print("Dongle not found.")
        time.sleep(1)


print("Connecting to VPN.")
subprocess.run(['echo "passwd" | sudo openconnect --protocol=anyconnect --authgroup=1 --user=boris.gaudard --passwd-on-stdin -b vpn.hefr.ch'], shell=True)
time.sleep(2)

try:
    mdynamix_ip = ni.ifaddresses('tun0')[ni.AF_INET][0]['addr'] 
    print(mdynamix_ip)
    trio.run(parent)
    # Update host file
    subprocess.run(["sudo sed -i '$ d' /etc/hosts"], shell=True)
    with open("/etc/hosts", "a") as file_object:
        file_object.write(f"{mdynamix_ip}  jetson4g\n")



    # Camera 
    subprocess.Popen(["cd /home/nvidia/Workspace/vcu/build && ./vcu"], shell=True)

    print("Launching...")
    # Roslaunch
    subprocess.run([f"export ROS_IP={mdynamix_ip} && export ROS_MASTER_URI=http://jetson4g:11311 && roslaunch mdynamix controller_all.launch && /home/nvidia/Workspace/vcu/build/vcu"], shell=True)

except KeyError:
    print("VPN interface not found.")



