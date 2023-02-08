import json

from std_msgs.msg import String


class Dictionary(String):

    def serialize(self, buff):
        self.data = json.dumps(self.data)
        super().serialize(buff)
        self.data = json.loads(self.data)

    def deserialize(self, str):
        cls = super().deserialize(str)
        cls.data = json.loads(cls.data)
        return cls

    def serialize_numpy(self, buff, numpy):
        return super().serialize(buff)

    def deserialize_numpy(self, str, numpy):
        return super().deserialize(str)
