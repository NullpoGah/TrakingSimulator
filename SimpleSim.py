import numpy as np
import rsa

import time
import xml.etree.ElementTree as ET
import argparse
import struct
import hmac
import hashlib
import socket
import logging

from pathlib import Path


def parse_gpx(file):
    tree = ET.parse(Path(file))
    root = tree.getroot()
    coords = root.findall(".//{http://www.topografix.com/GPX/1/1}trkpt")
    list_with_coords = []
    for coord in coords:
        elevation = coord.find('{http://www.topografix.com/GPX/1/1}ele').text
        lat = coord.get('lat')
        lon = coord.get('lon')
        # print(elevation, lat, lon)
        list_with_coords.append([float(lat), float(lon), float(elevation)])
    return list_with_coords


def split_host_port(string):
    if not string.rsplit(':', 1)[-1].isdigit():
        return (string, None)

    string = string.rsplit(':', 1)
    host = string[0]  
    port = int(string[1])
    return (host, port)


def get_logger(name) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    return logger


def destination_from_brng(phi1, lambda1, distance, brng):
    earthR = 6371000 # Earth R in meters
    phi2 = np.arcsin(np.sin(phi1)*np.cos(distance/earthR) + np.cos(phi1)*np.sin(distance/earthR)*np.cos(brng))
    lambda2 = lambda1 + np.arctan2(np.sin(brng)*np.sin(distance/earthR)*np.cos(phi1),np.cos(distance/earthR)-np.sin(phi1)*np.sin(phi2))
    return phi2, lambda2


def create_distances(coords, rate, velocity):
    earthR = 6371000 # Earth R in meters
    velocity = velocity*5/18 # convert km/h into m/s
    coords = np.asarray(coords)
    initShape = coords.shape
    coordinatesForSending = np.zeros((initShape[0], initShape[1]+2))
    coordinatesForSending[:,:-2] = coords
    changeInIndexes = 0
    for i in range(initShape[0]):
        if i == (initShape[0]-1):
            break
        lat1 = coords[i][0]
        lat2 = coords[i+1][0]
        lon1 = coords[i][1]
        lon2 = coords[i+1][1]
        phi1 = lat1*np.pi/180 # degrees to radians
        phi2 = lat2*np.pi/180 # degrees to radians
        lambda1 = lon1*np.pi/180 # degrees to radians
        lambda2 = lon2*np.pi/180 # degrees to radians
        deltaPhi = (lat2-lat1)*np.pi/180
        deltaLambda = (lon2-lon1)*np.pi/180
        a = np.power(np.sin(deltaPhi/2),2) + np.cos(phi1)*np.cos(phi2)*np.power(np.sin(deltaLambda/2),2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        distance = earthR * c # distance between two points

        # calculate bearing (course)
        y = np.sin(lambda2-lambda1)*np.cos(phi2)
        x = np.cos(phi1)*np.sin(phi2) - np.sin(phi1)*np.cos(phi2)*np.cos(lambda2-lambda1)
        Theta = np.arctan2(y, x)
        brng = (Theta*180/np.pi+360) % 360 # in degrees
        time = distance / velocity # in seconds
        timesToSend = np.floor(time / rate)
        coordinatesForSending[i+changeInIndexes][3] = np.round(brng, 4)
        coordinatesForSending[i+changeInIndexes][4] = np.round(np.random.normal(60, 2),2)
        arrToAppend = []
        for j in range(int(timesToSend)):
            if j == 0:
                continue
            distancePerRate = velocity * j
            la2, lo2 = destination_from_brng(phi1, lambda1, distancePerRate, brng)
            # append lat, lon, previous point elavation, bearing, calculate velocity
            arrToAppend.append([np.round(la2*180/np.pi, 8), np.round(lo2*180/np.pi, 8), coords[i][2], np.round(brng, 4), np.round(np.random.normal(60, 2),2)])
        # print(arrToAppend)
        if len(arrToAppend) == 0:
            continue
        coordinatesForSending = np.insert(coordinatesForSending, i+1+changeInIndexes, np.asarray(arrToAppend), axis=0)
        changeInIndexes = changeInIndexes + len(arrToAppend)
    # print(coordinatesForSending)
    return coordinatesForSending


def send_coords(coords, rate, address):
    """
    Message sender
    Message format:
        BYTES: <qqqddddd
            1. Vehicle ID - long long
            2. Unix timestamp - long long
            3. Vehicle type - long long
            4. Vehicle model - char[16]
            5. Latitude - double
            6. Longtitude - double
            7. Altitude - double
            8. Velocity - double
            9. Course(bearing in degrees) - double
    """
    logger = get_logger('sender')
    ID = int(np.round(np.random.normal(148822899220, 1000000000)))
    vType = 1
    vModel = 'Toyota Corolla'.encode('ascii')
    (pub, priv) = rsa.newkeys(1024)
    logger.info('Sender parsing adress')
    host, port = split_host_port(address)
    if port == None:
        logger.error('Port is not specified!')
        return
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except Exception as e:
        logger.error('Exception {} occured. Connection is not established, sender is closing.'.format(e))
        return
    for coord in coords:
        # ID = 148822899220
        
        msg = [
            ID, # long long
            time.time(), # long long
            vType, # long long
            vModel, 
            coord[0], #double
            coord[1], #double
            coord[2], #double
            coord[4], #double
            coord[3] #double
        ]
        msg = struct.pack('<qqq16sddddd',ID, int(round(time.time() * 1000)), vType, vModel, coord[0], coord[1], coord[2], coord[4], coord[3])
        # print(len(msg))
        crypto = rsa.encrypt(msg, pub)
        # print(len(crypto) == rsa.common.byte_size(pub.n))
        key = pub.n # modulus SUS -> int to bytes 128 little
        signature = hmac.new(key.to_bytes(128, 'little'), crypto, hashlib.sha256)
        finalMessage = crypto + signature.digest()
        sock.sendto(finalMessage, (host, port))
        logger.debug('Sender sent message')
        # print(len(finalMessage))
        # print(signature.digest_size)
        # message = rsa.decrypt(crypto, priv)
        # print(message)
        # print(struct.unpack('<qqq16sddddd', message))
        time.sleep(rate)
    # print(np.max(sizes))
    # print(rsa.common.byte_size(pub.n))
    # print(struct.calcsize('<qqq16sddddd'))
    


def main(file, rate, velocity, address):
    logger = get_logger('main')
    logger.info('Welcome to the metrics sender simulator!')
    logger.info('Starting parsing {}'.format(file))
    coords = parse_gpx(file)
    logger.info('Parsing finished! Creating nodes...')
    coords = create_distances(coords, rate, velocity)
    logger.info('Nodes created, sending metrics every {} seconds'.format(str(rate)))
    send_coords(coords, rate, address)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Input track and simulate sending realtime metrics from autonomous vehicle')
    parser.add_argument('address', default='127.0.0.1:5005', type=str, help='an address to send to via udp')
    parser.add_argument('-f','--file', default='TTK.gpx', help='track file')
    parser.add_argument('-r','--rate', type=int, default=1, choices=range(1,256), help='sending rate in seconds')
    parser.add_argument('--velocity', type=int, default=60, help='vehicle average velocity, km/h')
    args = parser.parse_args()
    main(args.file, args.rate, args.velocity, args.address)