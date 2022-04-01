import subprocess
import netifaces as ni
import trio
import time
import sys


PORT = 12345
mdynamix_ip = "0"
mdynamix_time = "00.00.00"
server_ip = "160.98.26.115"


async def sender(client_stream):
    msg = "Workstation".encode("UTF-8")
    await client_stream.send_all(msg)

async def receiver(client_stream):
    global mdynamix_ip
    global mdynamix_time
    async for data in client_stream:
        msg = data.decode("UTF-8")
        mdynamix_msg = msg.split("-")
        mdynamix_ip = mdynamix_msg[0]
        mdynamix_time = mdynamix_msg[1]
        print(f"Last update from Mdynamix received at {mdynamix_time}. Continue? (y/n): ", end = '')
        break


async def parent():
    client_stream = await trio.open_tcp_stream(server_ip, PORT)
    async with client_stream:
        async with trio.open_nursery() as nursery:
            nursery.start_soon(sender, client_stream)
            nursery.start_soon(receiver, client_stream)



network = -1. # 0 = School network via Ethernet, 1 = School network via WiFi, 2 = School network via VPN

# Ethernet
try:
    ip = ni.ifaddresses('enp0s31f6')[ni.AF_INET][0]['addr'] # Ethernet
    
    if ip[0:6] == '160.98' : # Si connecte au reseau de l'ecole
        print("Already connected to school network. Skipping VPN connection.")
        network = 0

    else :
        print("Connecting to VPN.")
        subprocess.run(['~/Desktop/anyconnect-linux64-4.8.03052/vpn/vpn connect vpn.hefr.ch -s < ~/Desktop/anyconnect-linux64-4.8.03052/vpn/cred'], shell=True)
        network = 2

except KeyError:
    print("Not connected via Ethernet. Trying WiFi.")
    # Wifi
    try:
        ip = ni.ifaddresses('wlp3s0')[ni.AF_INET][0]['addr'] # WiFi
        print("WiFi found.")

        if ip[0:6] == '160.98' : # Si connecte au reseau de l'ecole
            print("Already connected to school network. Skipping VPN connection.")
            network = 1

        else :
            print("Connecting to VPN.")
            subprocess.run(['~/Desktop/anyconnect-linux64-4.8.03052/vpn/vpn connect vpn.hefr.ch -s < ~/Desktop/anyconnect-linux64-4.8.03052/vpn/cred'], shell=True)
            network = 2

    except KeyError:
        print("Not connected via WiFi.")

    except:
        print("wlp3s0 not found")

except:
    print("enp0s31f6 not found")

# VPN
try:
    if network == 0: # Ethernet Interface
        workstation_ip = ni.ifaddresses('enp0s31f6')[ni.AF_INET][0]['addr'] # Check workstation IP.
    elif network == 1: # WiFi Interface
        workstation_ip = ni.ifaddresses('wlp3s0')[ni.AF_INET][0]['addr'] # Check workstation IP.
    elif network == 2: # VPN Interface
        workstation_ip = ni.ifaddresses('cscotun0')[ni.AF_INET][0]['addr'] # Check workstation IP.

    else:
        print("No internet connection found.")
        sys.exit()


    # Reveive Mdyn IP.
    trio.run(parent)
    if not input() == "y":
        print("Oh no...")
        sys.exit()

    # Update host file
    subprocess.run(["sudo sed -i '$ d' /etc/hosts"], shell=True)
    with open("/etc/hosts", "a") as file_object:
        file_object.write(f"{mdynamix_ip}  jetson4g\n")

    # Chromium
    #subprocess.Popen(["chromium --ignore-certificate-errors https://teleop.loxo.app:8080/"], shell=True)

    # Roslaunch
    subprocess.run([f"export ROS_IP={workstation_ip} && export ROS_MASTER_URI=http://jetson4g:11311 && roslaunch mdynamix logitech_g29.launch & chromium --ignore-certificate-errors https://teleop.loxo.app:8080/"], shell=True)

    
except KeyboardInterrupt:
    print("KeyboardInterrupt")











