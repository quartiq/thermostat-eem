
import miniconf
from gmqtt import Client as MqttClient
import json
import asyncio
import time
import csv
import numpy as np
from matplotlib import pyplot as plt
from pydwf import DwfLibrary, PyDwfError
from pydwf.utilities import openDwfDevice


# thermostat settings
PREFIX = "dt/sinara/thermostat-eem/80-1f-12-63-84-1b"
BROKER = "10.42.0.1"

NR_VALS = 100


class TelemetryReader:
    """ Helper utility to read telemetry. """

    @classmethod
    async def create(cls, prefix, broker, queue):
        """Create a connection to the broker and an MQTT device using it."""
        client = MqttClient(client_id='')
        await client.connect(broker)
        return cls(client, prefix, queue)

    def __init__(self, client, prefix, queue):
        """ Constructor. """
        self.client = client
        self._telemetry = []
        self.client.on_message = self.handle_telemetry
        self._telemetry_topic = f'{prefix}/telemetry'
        self.client.subscribe(self._telemetry_topic)
        self.queue = queue

    def handle_telemetry(self, _client, topic, payload, _qos, _properties):
        """ Handle incoming telemetry messages over MQTT. """
        assert topic == self._telemetry_topic
        self.queue.put_nowait(json.loads(payload))


async def get_tele(telemetry_queue):
    latest_values = await telemetry_queue.get()
    return latest_values['adc'][0]


def main():

    telemetry_queue = asyncio.LifoQueue()

    async def telemetry():
        await TelemetryReader.create(PREFIX, BROKER, telemetry_queue)
        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass

    telemetry_task = asyncio.Task(telemetry())

    f = open('data.csv', 'w')
    writer = csv.writer(f)
    writer.writerow(["raw thermostat samples from CH1 (second channel). 20Hz split between two ch (10Hz per ch). sinc5+sinc1, no postfilt."])
    temps = []

    async def collect():
        for nr in range(NR_VALS):
            data = await get_tele(telemetry_queue)
            print(f'data: {data}')
            writer.writerow([data])

        telemetry_task.cancel()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(collect())
    f.close()
    print("done")


if __name__ == "__main__":
    main()
