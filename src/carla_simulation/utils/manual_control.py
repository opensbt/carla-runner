#!/usr/bin/env python

# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a parser that takes the current keyboard input and returns
it as Xbox controller inputs. This enables manual control with which the ego
vehicle can be control by a human user via keyboard.
"""

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_e
    from pygame.locals import K_g
    from pygame.locals import K_i
    from pygame.locals import K_j
    from pygame.locals import K_k
    from pygame.locals import K_l
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_y
    from pygame.locals import K_1
    from pygame.locals import K_2
    from pygame.locals import K_3
    from pygame.locals import K_4
    from pygame.locals import K_8
    from pygame.locals import K_9
    from pygame.locals import K_0
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


def parse_keyboard_events_to_xbox():
    """
    Parse the current keyboard events to xbox controller inputs.
    """

    xbox_mapping = {"left_stick_X": 0.0,
                    "left_stick_Y": 0.0,
                    "right_stick_X": 0.0,
                    "right_stick_Y": 0.0,
                    "button_X": False,
                    "button_Y": False,
                    "button_A": False,
                    "button_B": False,
                    "button_L1": False,
                    "button_R1": False,
                    "button_L2": False,
                    "button_R2": False,
                    "button_select": False,
                    "button_home": False,
                    "button_start": False,
                    "dpad_up": False,
                    "dpad_down": False,
                    "dpad_left": False,
                    "dpad_right": False}

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return xbox_mapping

    keys = pygame.key.get_pressed()
    if keys[K_UP] or keys[K_w]:
        xbox_mapping["button_R2"] = True
    elif keys[K_DOWN] or keys[K_s]:
        xbox_mapping["button_L2"] = True
    if keys[K_LEFT] or keys[K_a]:
        xbox_mapping["left_stick_X"] = -1.0
    elif keys[K_RIGHT] or keys[K_d]:
        xbox_mapping["left_stick_X"] = 1.0
    if keys[K_g]:
        xbox_mapping["left_stick_Y"] = -1.0
    elif keys[K_t]:
        xbox_mapping["left_stick_Y"] = 1.0
    if keys[K_j]:
        xbox_mapping["right_stick_X"] = -1.0
    if keys[K_l]:
        xbox_mapping["right_stick_X"] = 1.0
    if keys[K_k]:
        xbox_mapping["right_stick_Y"] = -1.0
    if keys[K_i]:
        xbox_mapping["right_stick_Y"] = 1.0
    if keys[K_1]:
        xbox_mapping["dpad_up"] = True
    if keys[K_2]:
        xbox_mapping["dpad_down"] = True
    if keys[K_3]:
        xbox_mapping["dpad_left"] = True
    if keys[K_4]:
        xbox_mapping["dpad_right"] = True
    if keys[K_8]:
        xbox_mapping["button_select"] = True
    if keys[K_9]:
        xbox_mapping["button_home"] = True
    if keys[K_0]:
        xbox_mapping["button_start"] = True
    if keys[K_q]:
        xbox_mapping["button_L1"] = True
    if keys[K_e]:
        xbox_mapping["button_R1"] = True
    if keys[K_y]:
        xbox_mapping["button_X"] = True
    if keys[K_x]:
        xbox_mapping["button_Y"] = True
    if keys[K_c]:
        xbox_mapping["button_A"] = True
    if keys[K_v]:
        xbox_mapping["button_B"] = True

    return xbox_mapping
