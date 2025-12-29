#!/usr/bin/env python3
"""Test Firebase SSE listener"""
import requests
import json
import threading
import time

FIREBASE_DB_URL = "https://ve3axc-online-logger-default-rtdb.europe-west1.firebasedatabase.app"
STATION_ID = "kh18kbbh"

def listen():
    url = f"{FIREBASE_DB_URL}/stations/{STATION_ID}/target_freq.json"
    print(f"Connecting to: {url}")
    
    response = requests.get(
        url,
        headers={"Accept": "text/event-stream"},
        stream=True,
        timeout=30
    )
    print(f"Status: {response.status_code}")
    print(f"Headers: {dict(response.headers)}")
    
    print("\nListening for events...")
    for line in response.iter_lines():
        if line:
            print(f"RAW: {line}")
            line_str = line.decode('utf-8')
            if line_str.startswith('data:'):
                data_str = line_str[5:].strip()
                print(f"DATA: {data_str}")
                if data_str and data_str != 'null':
                    parsed = json.loads(data_str)
                    print(f"PARSED: {parsed}")
                    if isinstance(parsed, dict) and 'data' in parsed:
                        freq = parsed['data']
                        print(f">>> FREQUENCY: {freq} Hz = {freq/1e6:.6f} MHz")

if __name__ == "__main__":
    listen()
