#!/usr/bin/env python3


import inspect
import rospy


class TopicMirror:
    '''TopicMirror object subscribes to 'from' topic, converts its data using
    'fun' function if specified, then publishes converted data to 'to'
    topic. Types of topics are taken from 'fun' signature (annotations).'''


    def __init__(self, topicFrom: str, topicTo: str, fun):

        funSig = inspect.signature(fun)
        funTypeIn = next(iter(funSig.parameters.items()))[1].annotation
        funTypeOut = funSig.return_annotation

        if (funTypeIn is inspect._empty) or (funTypeOut is inspect._empty):
            raise Exception('Passed fun does not have any annotations')

        self.suber = rospy.Subscriber(
            topicFrom, funTypeIn, self.__cb_topicFrom)
        self.puber = rospy.Publisher(topicTo, funTypeOut, queue_size=10)
        self.fun = fun
        return


    def __cb_topicFrom(self, msg):
        '''Callback to use on 'from' topic.'''

        self.puber.publish(self.fun(msg))
        return
