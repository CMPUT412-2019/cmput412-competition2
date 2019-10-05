import rospy
from typing import Any, Callable, Optional


class SubscriberValue:
    def __init__(self, name, data_class, queue_size=1, transform=None):  # type: (str, Any, int, Optional[Callable[[Any], Any]) -> None
        self._subscriber = rospy.Subscriber(name, data_class, callback=self._callback, queue_size=queue_size)
        self._topic = name
        self._transform = transform
        self._value = None

    def _callback(self, message):
        if self._transform is None:
            self._value = message
        else:
            self._value = self._transform(message)

    def wait(self):
        while self._value is None:
            rospy.loginfo('Waiting for {}...'.format(self._topic))
            rospy.sleep(0.1)
        return self._value

    @property
    def value(self):
        return self._value