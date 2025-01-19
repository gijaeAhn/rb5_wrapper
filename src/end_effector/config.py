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
    MOTOR_Y_THRESHOLD = 63
    ENCODER_THRESHOLD = 0.008
    
    LOBBING = {
        'scoop': MotionConfig(
            position=[146, 46],
            stiffness=[20, 20],
            velocity_gain=[0.1, 0.1]
        ),
        'grab': MotionConfig(
            position=[97.17967272,66.71895146],
            stiffness=[80, 80],
            velocity_gain=[0.8, 0.8]
        ),
        'lob': MotionConfig(
            position=[50.014277 ,101.47677541],
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
    def update_position(cls, motion_type: str, new_position: List[float]):
        if motion_type in cls.LOBBING:
            cls.LOBBING[motion_type].position = new_position
            cls.save_config()