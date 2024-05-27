#!/usr/bin/env python3


import inspect
import typing

import rospy


class TopicMrr:
    '''Le Topic Mirror. Subscribes to 'from' topic, converts its data
    using 'converter' function if specified, then publishes converted
    data to 'to' topic. Types of topics are taken from 'converter'
    signature (annotations).'''


    def __init__(
            self,
            topic_from: str,
            topic_to: str,
            converter: typing.Callable[[typing.Any], typing.Any]):

        self.__fun = converter
        funSig = inspect.signature(self.__fun)
        funTypeIn = next(iter(funSig.parameters.items()))[1].annotation
        funTypeOut = funSig.return_annotation

        if (funTypeIn is inspect._empty) or (funTypeOut is inspect._empty):
            raise Exception('Passed fun does not have any annotations')

        def cb_topic_from(msg):
            '''Callback to use on 'from' topic.'''

            self.puber.publish(self.__fun(msg))
            return

        self.suber = rospy.Subscriber(
            topic_from, funTypeIn, cb_topic_from)
        self.puber = rospy.Publisher(topic_to, funTypeOut, queue_size=10)
        return
