import time, network
from BLE_CEEO import Yell
from machine import ADC
from mqtt import MQTTClient

class Song:

    def __init__(self):

        self.mqtt_broker = 'broker.emqx.io'
        self.port = 1883
        self.topic_sub = 'ME35-24/carlo'  
        self.topic_pub = 'ME35-24/song_status'  
        self.client = MQTTClient('song_status', self.mqtt_broker, self.port, keepalive=60)

        self.NoteOn = 0x90
        self.NoteOff = 0x80

        self.analog_pin = ADC(27)

        self.velocity = {'off': 0, 'pppp': 8, 'ppp': 20, 'pp': 31, 'p': 42, 'mp': 53,
                'mf': 64, 'f': 80, 'ff': 96, 'fff': 112, 'ffff': 127}

        self.pause_threshold = 410
        self.resume_threshold = 360

        self.p = Yell('Carla', verbose=True, type='midi')

        self.wifi()
        self.mqtt()
        self.p.connect_up()
        time.sleep(3)  # Allow time for setup
        
    def wifi(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect('Verizon_ZDW66Z','caught-pet3-wed')

        while wlan.ifconfig()[0] == '0.0.0.0':
            print('.', end=' ')
            time.sleep(1)

        print(wlan.ifconfig())

    def mqtt(self):
        self.client.connect()
        print('Connected to %s MQTT broker' % (self.mqtt_broker))
        self.client.set_callback(self.callback)  # Set the callback if anything is read
        self.client.subscribe(self.topic_sub.encode())  # Subscribe to the topic

    def callback(self, topic, msg):
        topic, msg = topic.decode(), msg.decode()
        print(f'Received message: {msg}')

    def ticks_to_ms(self, ticks, ticks_per_qnote=480, tempo_us_per_qnote=600000):  # 600 ms per quarter note at 100 BPM
        ms_per_qnote = tempo_us_per_qnote / 1000  # Convert to milliseconds
        return (ticks / ticks_per_qnote) * ms_per_qnote

    def parse_midi_data(self, filename):
        midi_events = []
        previous_time = 0
        with open(filename, 'r') as file:
            for line in file:
                parts = line.split(", ")
                if len(parts) < 3:
                    continue  # Skip invalid lines
                
                # Extract tick time
                time_part = parts[0].split(":")[1].strip().split()[0]
                ticks = int(time_part)

                # Extract note type (On or Off) and note number
                note_info = parts[1].split(": ")
                note_type = note_info[0].strip()
                note = int(note_info[1].strip())

                # Extract velocity
                velocity_info = parts[2].split(": ")
                velocity_val = int(velocity_info[1].strip())

                if note_type == "Note ON":
                    cmd = self.NoteOn
                elif note_type == "Note OFF":
                    cmd = self.NoteOff
                else:
                    continue  # Skip invalid lines
                
                # Calculate time delay from ticks
                time_ms = self.ticks_to_ms(ticks - previous_time)
                previous_time = ticks

                midi_events.append((time_ms, note, velocity_val, cmd))
        
        return midi_events

    # Function to create MIDI payload
    def create_midi_payload(self, cmd, channel, note, velocity):
        timestamp_ms = time.ticks_ms()
        tsM = (timestamp_ms >> 7 & 0b111111) | 0x80
        tsL = 0x80 | (timestamp_ms & 0b1111111)
        c = cmd | (0x0F & channel)
        return bytes([tsM, tsL, c, note, velocity])

    # Function to play a song from a specific index
    def play_song(self, filename, start_index=0):
        print(f"Playing song from {filename}")
        midi_events = self.parse_midi_data(filename)

        # Resume from the provided index
        for event_index, event in enumerate(midi_events[start_index:], start=start_index):
            time_ms, note, vel, cmd = event
            payload = self.create_midi_payload(cmd, 0, note, vel)
            self.p.send(payload)
            time.sleep(time_ms / 1000)  # Convert ms to seconds

            # Check if we need to pause, return the current event index
            analog_value = self.analog_pin.read_u16()
            if analog_value > self.pause_threshold:
                print("Pausing song...")
                return event_index  # Return current index to resume later
        
        return None  # Song finished


    def play(self):
        
        song_files = ["jingle.txt", "deck.txt", "merry_xmas.txt"]

        
        song_index = 0
        current_event_index = 0
        is_playing = False

    
        total_songs = len(song_files)

        while song_index < total_songs:
            # Read photoresistor value
            analog_value = self.analog_pin.read_u16()

            if analog_value < self.resume_threshold and not is_playing:
                # Start playing the song from the current position
                print(f"Resuming song {song_files[song_index]} at event {current_event_index}")
                self.client.publish(self.topic_pub, f"Resuming song {song_files[song_index]} at event {current_event_index}")
                current_event_index = self.play_song(song_files[song_index], start_index=current_event_index)
                
                if current_event_index is None:
                    # Song finished, move to the next song
                    song_index += 1  # Move to the next song in the list
                    current_event_index = 0  # Reset event index for the next song
                    self.client.publish(self.topic_pub, f"Finished song {song_files[song_index - 1]}")
                is_playing = True

            elif analog_value > self.pause_threshold and is_playing:
                # Pause the song
                print("Song paused.")
                self.client.publish(self.topic_pub, "Song paused")
                is_playing = False

            time.sleep(0.1)  
        if song_index == total_songs:
            print("All songs have finished playing. Disconnecting...")
            self.client.publish(self.topic_pub, "All songs have finished playing")
            self.p.disconnect()

song = Song()

song.play()
