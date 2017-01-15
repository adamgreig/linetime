import yaml
raw_packets = yaml.load(open("pktlog"))
stripped_packets = []
for packet in raw_packets:
    # clear the ip4 ID and ip4 checksum
    packet[18] = packet[19] = 0
    packet[24] = packet[25] = 0
    p = tuple(packet)
    if p not in stripped_packets:
        stripped_packets.append(p)

for packet in stripped_packets:
    print(" ".join(["{:02X}".format(x) for x in packet]))
