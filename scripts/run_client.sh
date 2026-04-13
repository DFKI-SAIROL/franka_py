#!/bin/bash
pixi run -e humble
python franka_client/franka_client/client.py
