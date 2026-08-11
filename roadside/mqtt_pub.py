from __future__ import print_function

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


class MqttPublisher(object):
    def __init__(self, config):
        self.enabled = bool(config.get("enabled", False))
        self.config = config
        self.client = None

    def connect(self):
        if not self.enabled:
            return
        if mqtt is None:
            raise RuntimeError("paho-mqtt is not installed")
        self.client = mqtt.Client()
        self.client.connect(self.config.get("host", "127.0.0.1"),
                            int(self.config.get("port", 1883)), 60)
        self.client.loop_start()

    def publish(self, topic, payload):
        if self.enabled and self.client is not None:
            self.client.publish(topic, payload)

    def close(self):
        if self.client is not None:
            self.client.loop_stop()
            self.client.disconnect()
            self.client = None
