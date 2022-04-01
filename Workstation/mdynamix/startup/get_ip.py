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
        break


async def parent():
    client_stream = await trio.open_tcp_stream(server_ip, PORT)
    async with client_stream:
        async with trio.open_nursery() as nursery:
            nursery.start_soon(sender, client_stream)
            nursery.start_soon(receiver, client_stream)



# VPN
try:
    # Reveive Mdyn IP.
    trio.run(parent)

    print(f"Mdynamix last known ip is {mdynamix_ip}.")


except KeyboardInterrupt:
    print("KeyboardInterrupt")











