#!/usr/bin/env python3
"""
DH Robotics Dexterous Hand Modbus TCP Interface
================================================
This module provides a Python Modbus TCP client for direct low-latency control of 
DH Robotics Dexterous Multi-Finger Hand hardware using `pymodbus`:
- Controls individual finger target positions (Thumb, Index, Middle, Ring, Pinky).
- Configures grasping force thresholds and closing/opening velocities.
- Reads real-time finger position & force feedback registers.

Usage:
    from src.hardware.dh_hand_modbus import DHHandModbusClient
    hand = DHHandModbusClient(host="192.168.1.100", port=502)
    hand.connect()
    hand.set_finger_positions([500, 500, 500, 0, 0]) # Close first 3 fingers (Pinch Grasp)
"""

import time
import logging
from typing import List, Optional, Dict

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:
    ModbusTcpClient = None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DHHandModbus")

class DHHandModbusClient:
    """Modbus TCP driver interface for DH Robotics Multi-Finger Dexterous Hand."""

    # Register Map Definitions (DH Modbus Protocol Standard)
    REG_HAND_INIT = 0x0100       # 1: Activate/Initialize Hand
    REG_FORCE_LIMIT = 0x0101      # 20 - 100 (% max torque)
    REG_SPEED_LIMIT = 0x0102      # 1 - 100 (% max joint speed)
    REG_FINGER_POS_BASE = 0x0103  # 0x0103 (Thumb), 0x0104 (Index), 0x0105 (Middle), 0x0106 (Ring), 0x0107 (Pinky)
    
    # Read-only Feedback Registers
    REG_READ_STATE_BASE = 0x0200  # Finger actual positions & state status

    NUM_FINGERS = 5

    def __init__(self, host: str = "192.168.1.100", port: int = 502, slave_id: int = 1):
        self.host = host
        self.port = port
        self.slave_id = slave_id
        self.client: Optional[ModbusTcpClient] = None
        self._is_connected = False

    def connect(self) -> bool:
        """Establishes Modbus TCP connection with DH Hand controller."""
        if ModbusTcpClient is None:
            logger.error("`pymodbus` is not installed. Install via: pip install pymodbus")
            return False

        logger.info(f"Connecting to DH Dexterous Hand at {self.host}:{self.port} (Slave ID: {self.slave_id})...")
        self.client = ModbusTcpClient(self.host, port=self.port)
        self._is_connected = self.client.connect()

        if self._is_connected:
            logger.info("Connected successfully to DH Hand controller.")
            self._initialize_hand()
        else:
            logger.warning(f"Failed to connect to DH Hand at {self.host}:{self.port}. (Running in mock mode if disconnected).")
        
        return self._is_connected

    def _initialize_hand(self):
        """Sends initialization pulse to enable finger motor drivers."""
        if not self._is_connected or not self.client:
            return
        try:
            # Write 1 to REG_HAND_INIT
            self.client.write_register(self.REG_HAND_INIT, 1, slave=self.slave_id)
            time.sleep(0.5)
            # Default speed (80%) and force (70%)
            self.client.write_register(self.REG_SPEED_LIMIT, 80, slave=self.slave_id)
            self.client.write_register(self.REG_FORCE_LIMIT, 70, slave=self.slave_id)
            logger.info("DH Hand initialized with default speed 80% and force 70%.")
        except Exception as e:
            logger.error(f"Error initializing hand: {e}")

    def set_finger_positions(self, positions: List[int]) -> bool:
        """
        Sets target position for 5 fingers (0 = fully open, 1000 = fully closed).
        
        Args:
            positions: List of 5 integers [Thumb, Index, Middle, Ring, Pinky] in range [0, 1000].
        """
        if len(positions) != self.NUM_FINGERS:
            logger.error(f"Positions must contain exactly {self.NUM_FINGERS} elements.")
            return False

        clamped = [max(0, min(1000, p)) for p in positions]

        if not self._is_connected or not self.client:
            logger.debug(f"[MOCK HAND WRITE] Target Finger Positions: {clamped}")
            return True

        try:
            # Write multiple registers starting from REG_FINGER_POS_BASE
            self.client.write_registers(self.REG_FINGER_POS_BASE, clamped, slave=self.slave_id)
            return True
        except Exception as e:
            logger.error(f"Modbus write exception: {e}")
            return False

    def get_finger_states(self) -> Dict[str, List[int]]:
        """Reads current position and status feedback registers from the hand."""
        if not self._is_connected or not self.client:
            return {"positions": [0, 0, 0, 0, 0], "connected": False}

        try:
            response = self.client.read_holding_registers(self.REG_READ_STATE_BASE, count=self.NUM_FINGERS, slave=self.slave_id)
            if not response.isError():
                return {"positions": response.registers, "connected": True}
        except Exception as e:
            logger.error(f"Modbus read exception: {e}")
            
        return {"positions": [0, 0, 0, 0, 0], "connected": False}

    def disconnect(self):
        """Closes Modbus connection."""
        if self.client and self._is_connected:
            self.client.close()
            self._is_connected = False
            logger.info("DH Hand Modbus connection closed.")

if __name__ == "__main__":
    # Quick standalone testing script
    client = DHHandModbusClient(host="192.168.1.100", port=502)
    connected = client.connect()
    
    print("\n--- DH Hand Modbus Test ---")
    print("Commanding Pinch Grasp (Closing Thumb, Index, Middle)...")
    client.set_finger_positions([800, 800, 800, 0, 0])
    time.sleep(1.0)
    
    print("Commanding Full Hand Open...")
    client.set_finger_positions([0, 0, 0, 0, 0])
    client.disconnect()
