#!/bin/python

import socket


def receive_udp_broadcast(port=2002, buffer_size=1024):
    # Create a new socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Allow reusing the same address
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Set socket options for broadcasting
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # Bind to the desired port
    sock.bind(("", port))

    while True:
        # Receive data from the socket
        data, addr = sock.recvfrom(buffer_size)
        message = data.decode("utf-8")

        # Check if the message ends with a newline
        if message.endswith("\n"):
            print(f"Received message from {addr}: {message.strip()}")


if __name__ == "__main__":
    receive_udp_broadcast()
