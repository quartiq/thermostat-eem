#!/usr/bin/python3
"""Thermostat EEM utilities"""

# Sample period in seconds for all channels.
SAMPLE_PERIOD = 1 / 1007  # ADC ODR 1007 split between ADC channels. But with the ZOH the IIR updates at full rate.
