from __future__ import print_function

import os
import unittest

import roadside.mqtt_pub as mqtt_pub


class _FakeClient(object):
    def __init__(self,client_id=""):
        self.client_id=client_id;self.calls=[];self.on_message=None
    def username_pw_set(self,user,password):self.calls.append(("auth",user,password))
    def connect(self,host,port,keepalive):self.calls.append(("connect",host,port,keepalive))
    def loop_start(self):self.calls.append(("loop_start",))
    def subscribe(self,topic,qos=0):self.calls.append(("subscribe",topic,qos))
    def publish(self,topic,payload,qos=0):self.calls.append(("publish",topic,payload,qos));return "info"
    def loop_stop(self):self.calls.append(("loop_stop",))
    def disconnect(self):self.calls.append(("disconnect",))


class _FakeMqtt(object):
    def __init__(self):self.client=None
    def Client(self,client_id=""):
        self.client=_FakeClient(client_id);return self.client


class MqttPublisherTest(unittest.TestCase):
    def setUp(self):self.original=mqtt_pub.mqtt;self.fake=_FakeMqtt();mqtt_pub.mqtt=self.fake
    def tearDown(self):mqtt_pub.mqtt=self.original

    def test_connect_auth_subscribe_and_qos2_publish(self):
        os.environ["TEST_RSU_PASSWORD"]="secret"
        try:
            publisher=mqtt_pub.MqttPublisher({"enabled":True,"host":"broker",
                "port":1883,"client_id":"mec","username":"user",
                "password_env":"TEST_RSU_PASSWORD","qos":2,
                "response_topic":"command///res/#"})
            publisher.connect();self.assertEqual("info",publisher.publish("t","p"))
            calls=self.fake.client.calls
            self.assertIn(("auth","user","secret"),calls)
            self.assertIn(("subscribe","command///res/#",2),calls)
            self.assertIn(("publish","t","p",2),calls)
            publisher.close()
        finally:os.environ.pop("TEST_RSU_PASSWORD",None)

    def test_missing_named_password_fails_closed(self):
        os.environ.pop("DEFINITELY_MISSING_RSU_PASSWORD",None)
        publisher=mqtt_pub.MqttPublisher({"enabled":True,"username":"user",
            "password_env":"DEFINITELY_MISSING_RSU_PASSWORD"})
        with self.assertRaises(RuntimeError):publisher.connect()


if __name__=="__main__":unittest.main()
