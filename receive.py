#!/usr/bin/env python3
import serial, time, datetime, sys, math
import argparse

I_pin = 4
V_pin = 0
MAINS_VPP = 170*2

Iref_cal = 501
Vref_cal = 536
I_norm = 15.5
#samples = 30

def main(port, baud, log_fn):
    ser = open_serial(port, baud)
    log = open_log(log_fn)

    while True:
        update(ser,log)

def open_serial(port, baud):
    print('Opening serial port {}'.format(port))
    return serial.Serial(port,baud)

def open_log(fn):
    log = None
    try:
        print('Opening log file {}'.format(fn))
        log = open(fn, 'a+')
        #log.write("time:sensor_id:voltage:current:avg_power\n")
        #log.flush()
    return log
    
def fetch_packet(ser):
    if ser.read() == b'~':
        len_msb = ord(ser.read())
        len_lsb = ord(ser.read())
        packet_len = (len_lsb + (len_msb << 8)) + 1
        return ser.read(packet_len)
    else:
        return None

def parse_packet(raw_bytes):
    app_id = raw_bytes[0]

    #print('APP_ID: {}'.format(app_id))
    if not app_id == 0x83:
        return None

    addr_16 = (raw_bytes[1] << 8) + raw_bytes[2]

    #rssi = raw_bytes[3]
    #addr_broadcast = ((raw_bytes[4] >> 1) & 0x01) == 1
    #pan_broadcast = ((raw_bytes[4] >> 2) & 0x01) == 1

    total_samples = raw_bytes[5]
    channel_indicator_high = raw_bytes[6]
    channel_indicator_low = raw_bytes[7]

    #local_checksum = int(app_id, 16) + addr_msb + addr_lsb + rssi + p[4] + total_samples + channel_indicator_high + channel_indicator_low

    digital_samples = []
    analog_samples = []

    for n in range(total_samples):
        dataD = [-1] * 9
        digital_channels = channel_indicator_low
        digital = 0

        #for i in range(len(dataD)):
        #    if (digital_channels & 1) == 1:
        #        dataD[i] = 0
        #        digital = 1
        #    digital_channels = digital_channels >> 1

        #if (channel_indicator_high & 1) == 1:
        #    dataD[8] = 0
        #    digital = 1

        #if digital:
        #    dig_msb = raw_bytes[8]
        #    dig_lsb = raw_bytes[9]
        #    local_checksum += dig_msb + dig_lsb
        #    dig = (dig_msb << 8) + dig_lsb
        #    for i in range(len(dataD)):
        #        if dataD[i] == 0:
        #            dataD[i] = dig & 1
        #        dig = dig >> 1

        #digital_samples.append(dataD)

        analog_count = None
        dataA = [-1] * 6
        analog_channels = channel_indicator_high >> 1
        valid_analog = 0

        for i in range(len(dataA)):
            if ((analog_channels >> i) & 1) == 1:
                valid_analog += 1

        for i in range(len(dataA)):
            if (analog_channels & 1) == 1:
                analogchan = 0
                for j in range(i):
                    if ((channel_indicator_high >> (j+1)) & 1) == 1:
                        analogchan += 1
                dataA_msb = raw_bytes[8 + valid_analog * n * 2 + analogchan*2]
                dataA_lsb = raw_bytes[8 + valid_analog * n * 2 + analogchan*2 + 1]
                #local_checksum += dataA_msb + dataA_lsb
                dataA[i] = ((dataA_msb << 8) + dataA_lsb)

                analog_count = i
            analog_channels = analog_channels >> 1
        analog_samples.append(dataA)
        
    return locals()

def update(ser, log):
    packet = fetch_packet(ser)
    if not packet:
        #print('Packet not found')
        return
    #print('Packet found: {}'.format(packet))

    data = parse_packet(packet)
    #print(data)

    result = calculate_data(data)
    #print(result)
    if not result == None:
        log.write("{}\n".format(str(result)))
        log.flush()

def calculate_data(data):
    v_data = [-1] * (len(data['analog_samples']) - 1)
    i_data = [-1] * (len(data['analog_samples']) - 1)

    for i in range(len(v_data)):
        v_data[i] = data['analog_samples'][i+1][V_pin]
        i_data[i] = data['analog_samples'][i+1][I_pin]

    #print("V:{}".format(v_data))
    #print("I:{}".format(i_data))

    min_v = 1024
    max_v = 0

    for idx in range(len(v_data)):
        v_data[idx] = data['analog_samples'][idx+1][V_pin]
        i_data[idx] = data['analog_samples'][idx+1][I_pin]

        if (min_v > v_data[idx]):
            min_v = v_data[idx]
        if (max_v < v_data[idx]):
            max_v = v_data[idx]

    #max_v = max(v_data)
    #min_v = min(v_data)

    v_avg = (max_v + min_v) / 2
    v_pp = max_v - min_v

    #print(v_pp)
    if v_pp == 0:
        return None

    for i in range(len(v_data)):
        v_data[i] -= v_avg
        v_data[i] = (v_data[i] * MAINS_VPP) / Vref_cal

    for i in range(len(i_data)):
        i_data[i] -= Iref_cal
        i_data[i] /= I_norm

    #print("V2:{}".format(v_data))
    #print("I2:{}".format(i_data))

    w_data = [0] * len(v_data)
    for i in range(len(w_data)):
        w_data[i] = v_data[i] * i_data[i]

    i_avg = 0
    for i in range(len(i_data)):
        i_avg += abs(i_data[i])
    i_avg /= len(i_data)

    w_avg = 0
    for i in range(len(w_data)):
        w_avg += abs(w_data[i])
    w_avg /= len(w_data)

    #print("V:{}".format(v_data))
    #print("I:{}".format(i_data))

    #print("Average Current:{}".format(i_avg))
    #print("Average Power:{}".format(w_avg))

    v_rms = (max(v_data) - min(v_data)) / (2*math.sqrt(2))
    w_avg = i_avg*v_rms
    #v_rms = 120
    
    return {'time':int(time.time()),'addr':data['addr_16'],'voltage':v_rms,'current':i_avg,'power':w_avg}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XBee Receiver", prog=sys.argv[0])
    parser.add_argument("-p", "--port", help="Serial port device", default='/dev/ttyUSB0')
    parser.add_argument("-b", "--baud", help="Serial port baud rate", default=9600)
    parser.add_argument("-l", "--log", help="Log filename", default='data/power.data')
    args = parser.parse_args()
    main(args.port, args.baud, args.log)
