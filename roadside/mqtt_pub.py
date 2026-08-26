from __future__ import print_function

import os

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


class MqttPublisher(object):
    def __init__(self, config):
        self.enabled = bool(config.get("enabled", False))
        self.config = config
        self.client = None

    def _on_message(self,client,userdata,message):
        try:payload=message.payload.decode("utf-8","replace")
        except Exception:payload=str(message.payload)
        print("[MQTT RX] topic=%s payload=%s"%(message.topic,payload))

    def connect(self):
        if not self.enabled:
            return
        if mqtt is None:
            raise RuntimeError("paho-mqtt is not installed")
        self.client = mqtt.Client(client_id=str(self.config.get("client_id","roadside-mec")))
        username=self.config.get("username");password=self.config.get("password")
        password_env=self.config.get("password_env")
        if password is None and password_env:password=os.environ.get(str(password_env))
        if username and password_env and password is None:
            raise RuntimeError("MQTT password environment variable %s is not set"%password_env)
        if username:self.client.username_pw_set(str(username),None if password is None else str(password))
        self.client.on_message=self._on_message
        self.client.connect(self.config.get("host", "127.0.0.1"),
                            int(self.config.get("port", 1883)),
                            int(self.config.get("keepalive",60)))
        self.client.loop_start()
        response_topic=self.config.get("response_topic")
        if response_topic:self.client.subscribe(str(response_topic),qos=int(self.config.get("qos",2)))
        print("[MQTT] Connected broker=%s:%s qos=%s"%(
            self.config.get("host","127.0.0.1"),self.config.get("port",1883),
            self.config.get("qos",2)))

    def publish(self, topic, payload):
        if self.enabled and self.client is not None:
            info=self.client.publish(topic,payload,qos=int(self.config.get("qos",2)))
            return info
        return None

    def close(self):
        if self.client is not None:
            self.client.loop_stop()
            self.client.disconnect()
            self.client = None
