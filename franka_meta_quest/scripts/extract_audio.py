#!/usr/bin/env python3
import sys
import wave
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

def extract_audio(bag_file, output_wav='output.wav', topic_name='/franka_right/audio'):
    print(f"Opening bag {bag_file}...")
    conn = sqlite3.connect(bag_file)
    cursor = conn.cursor()
    
    # Get topic ID
    cursor.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
    topic_row = cursor.fetchone()
    if not topic_row:
        print(f"Topic {topic_name} not found in bag.")
        return
        
    topic_id, msg_type = topic_row
    
    print(f"Reading audio chunks from topic {topic_name}...")
    cursor.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (topic_id,))
    
    # Get the message class
    msg_class = get_message(msg_type)
    
    frames = bytearray()
    
    for row in cursor.fetchall():
        try:
            # Deserialize
            msg = deserialize_message(row[0], msg_class)
            # data is a UInt8MultiArray
            # std_msgs.msg.UInt8MultiArray
            frames.extend(bytes(msg.data))
        except Exception as e:
            print(f"Error reading message: {e}")
            
    if len(frames) == 0:
        print("No audio data found.")
        return
        
    print(f"Writing {len(frames)} bytes to {output_wav}...")
    with wave.open(output_wav, 'wb') as wav_file:
        wav_file.setnchannels(1) # Mono
        wav_file.setsampwidth(2) # 16-bit (2 bytes)
        wav_file.setframerate(48000) # Assuming scrcpy default 48kHz
        wav_file.writeframes(frames)
    
    print("Done!")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_audio.py <path_to_bag_file.db3>")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    extract_audio(bag_path)
