import socket
import struct

so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
si = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
si.settimeout(0.005)
si.bind(('', 9090))

sent = 0
responses = 0
timeouts = 0
while True:
    so.sendto(struct.pack("I", sent), ("192.168.2.202", 9090))
    sent += 1
    try:
        m = struct.unpack("I", si.recv(4))[0]
        responses += 1
    except socket.timeout:
        timeouts += 1
    if sent % 100 == 0:
        print("sent {}, received {}, lost {} ({}%)"
              .format(sent, responses, timeouts, (100*timeouts)//sent))
