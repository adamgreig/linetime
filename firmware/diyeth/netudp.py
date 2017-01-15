import socket
import struct

so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
si = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
si.settimeout(0.005)
si.bind(('', 9090))

sent = 0
responses = 0
timeouts = 0
errors = 0
m = 0

while True:
    so.sendto(struct.pack("I", 0x12345678), ("192.168.2.202", 9090))
    sent += 1
    try:
        m = struct.unpack("I", si.recv(4))[0]
        if abs(m - sent) < 20 or (100 - m) < 20:
            responses += 1
        else:
            errors += 1
            print("sent {} received {}".format(sent, m))
    except socket.timeout:
        timeouts += 1
    if sent % 100 == 0:
        print("sent {:03}, received {:03}, lost {:03}, errors {:02}"
              .format(sent, responses, timeouts, errors))
        sent = responses = timeouts = errors = 0
