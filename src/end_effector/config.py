from dataclasses import dataclass
from typing import List
import json
import os

@dataclass
class MotionConfig:
    position: List[float]
    stiffness: List[float]
    velocity_gain: List[float]

    def to_dict(self):
        return {
            'position': self.position,
            'stiffness': self.stiffness,
            'velocity_gain': self.velocity_gain
        }
    
    @classmethod
    def from_dict(cls, data):
        return cls(**data)

class eeConfig:
    CONFIG_FILE = 'eeConfig.json'

    _doOnce : bool = False
    
    LOBBING = {
        'scoop': MotionConfig(
            position=[151.67, 37.69],
            stiffness=[20, 20],
            velocity_gain=[0.1, 0.1]
        ),
        'grab': MotionConfig(
            position=[100.21, 40.26],
            stiffness=[80, 80],
            velocity_gain=[0.8, 0.8]
        ),
        'lob': MotionConfig(
            position=[64.03, 76.77],
            stiffness=[80, 80],
            velocity_gain=[0.6, 0.6]
        )
    }

    TIMING = {
        'lobbing_duration': 5.0,
        'max_grab_duration': 7.0,
        'initial_delay': 1.0
    }
    
    @classmethod
    def save_config(cls):
        config_data = {
            'lobbing': {
                key: value.to_dict() for key, value in cls.LOBBING.items()
            }
        }
        with open(cls.CONFIG_FILE, 'w') as f:
            json.dump(config_data, f, indent=4)
    
    @classmethod
    def load_config(cls):
       if os.path.exists(cls.CONFIG_FILE):
           try:
               with open(cls.CONFIG_FILE, 'r') as f:
                   data = json.load(f)
                   cls.LOBBING = {
                       key: MotionConfig.from_dict(value) 
                       for key, value in data['lobbing'].items()
                   }
           except Exception as e:
               print(f"Error loading config file: {e}")
               print("Using default settings and creating new config file.")
               cls.save_config()
       else:
           print("Config file not found. Creating new file with default settings.")
           cls.save_config()

    
    @classmethod
    def update_position(cls, motion_type: str, new_position: List[float]):
        if motion_type in cls.LOBBING:
            cls.LOBBING[motion_type].position = new_position
            cls.save_config()