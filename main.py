
import machine, utime, neopixel, ujson, ubinascii, micropython,network,esp, gc
import _thread as th
from umqtt.robust import MQTTClient

# Neopixel Pin
p = machine.Pin

esp.osdebug(None)
gc.collect()

#machine.freq(240000000)

# Network Config
ssid = 'ssid'
pw = 'pass'

# MQTT Config
mqtt_server = '192.168.0.60'
client_id = ubinascii.hexlify(machine.unique_id())
topic_sub = b'/trains/#'


# Connect Wifi
station = network.WLAN(network.STA_IF)
station.active(True)
station.connect(ssid, pw)

while station.isconnected() == False:
  pass

print('Connection successful')
print(station.ifconfig())

def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def sub_cb(topic, msg):
    # Callback function for MQTT. Convert message into object,
    # parse into specific variable based on input data
    message = (topic.decode('utf-8'), msg.decode('utf-8'))
    try:
        msgtype = message[0].split('/')[4].split('-')[0]
    except:
        print('parsing error')

    if msgtype == 'QTSIG':
        headid = message[0].replace("/trains/track/turnout/QTSIG-", "")
        state = message[1]
        try:
            lightstate[headid]['state'] = state
        except:
            print('WARNING: unable to match message for ' + headid + ' to key')
    if msgtype == 'QTSWI':
        headid = message[0].replace("/trains/track/turnout/QTSWI-", "")
        state = message[1]
        try:
            servos[headid]['state'] = state
        except:
            print('WARNING: unable to match message for ' + headid + ' to key')

def connect_and_subscribe(client_id):
  global mqtt_server, topic_sub
  client = MQTTClient(client_id, mqtt_server)
  client.set_callback(sub_cb)
  client.connect()
  client.subscribe(topic_sub)
  print('Connected to %s MQTT broker, subscribed to %s topic' % (mqtt_server, topic_sub))
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  utime.sleep(10)
  machine.reset()


def signal (sig_num):
    np = neopixel.NeoPixel(p(signals[sig_num]['pin']), signals[sig_num]['lights'])
    colour ={
                'R': Red,
                'Y': Yellow,
                'G': Green,
                'O': Off,
                'L': Lunar,
                }
    while True:
        # Build another dictionary based on the signal name.
        newDict = {k:v for (k,v) in lightstate.items() if k.startswith(sig_num)}
        # build the local np object based on the global vars state
        for key, value in newDict.items():
            index = int(value['address'])
            aspect = key.split('-')[3]
            state = value['state']
            if state == 'THROWN':
                np[index] = colour.get(aspect)
            if state == 'CLOSED':
                np[index] = (0,0,0) # Turn off
        #write to the neopixel
        np.write()

def xover (xover_num):
    pin = machine.Pin(servos[xover_num]['pin'])
    speed = servos[xover_num]['speed']
    pwm = machine.PWM(pin)
    pwm.freq(50)
    pwm.duty(servos[xover_num]['Normal'])
    if servos[xover_num]['state'] == 'THROWN':
        pos = servos[xover_num]['Reverse']
    if servos[xover_num]['state'] == 'CLOSED':
        pos = servos[xover_num]['Normal']
    while True:
        # Determine if we need to make a change
        if servos[xover_num]['state'] == 'THROWN':
            ep = servos[xover_num]['Reverse']
        if servos[xover_num]['state'] == 'CLOSED':
            ep = servos[xover_num]['Normal']
        if pos < ep: # increment the number
            tpc = '/trains/track/turnout/QTRS-'+ xover_num + '-Nor'
            baton.acquire()
            pub.publish(tpc, 'CLOSED')
            baton.release()
            while pos < ep:
                pos = pos + 1
                #baton.acquire()
                pwm.duty(pos)
                #baton.release()
                utime.sleep_ms(speed)
            tpc = '/trains/track/turnout/QTRS-'+ xover_num + '-Rev'
            baton.acquire()
            pub.publish(tpc, 'THROWN')
            baton.release()
        if ep < pos: # decrement the number
            tpc = '/trains/track/turnout/QTRS-'+ xover_num + '-Rev'
            baton.acquire()
            pub.publish(tpc, 'CLOSED')
            baton.release()
            while ep < pos:
                pos = pos - 1
                #baton.acquire()
                pwm.duty(pos)
                #baton.release()
                utime.sleep_ms(speed)
            tpc = '/trains/track/turnout/QTRS-'+ xover_num + '-Nor'
            baton.acquire()
            pub.publish(tpc, 'THROWN')
            baton.release()

baton = th.allocate_lock()

# Connect to Broker

try:
  client = connect_and_subscribe(b'sub_' + client_id)
  pub = connect_and_subscribe(b'pub_' + client_id)

except OSError as e:
  restart_and_reconnect()

# Load config files for Signal Pins, Signal State, and XOvers

f = open('signal_state.json')
lightstate = ujson.load(f)

f = open('signal_pins.json')
signals = ujson.load(f)

f = open('servo_pins.json')
servos = ujson.load(f)

# Colour Vars

Yellow = (127,40,0)
Red = (127,0,0)
Green = (0,127,0)
Off = (0,0,0)
Lunar= (127,127,127)

print('spawning threads')

for s in signals:
    print('   signal: ' + s)
    th.start_new_thread(signal, (s,))

for s in servos:
    print('   servo: ' + s)
    th.start_new_thread(xover, (s,))

print('startup complete')

while True:
    #Wait for Messages from the topic - callback function updates vars as required.
    try:
        client.wait_msg()
    except:
        print('something went wrong waiting for mqtt')
