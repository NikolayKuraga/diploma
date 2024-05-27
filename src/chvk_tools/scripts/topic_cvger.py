#!/usr/bin/env python3


import collections
import inspect
import typing


import rospy


class TopicCvger:
    '''Le Topic Convergencer (I don't have personal life). Subscribes to
    several "from"-topics, does conversion, publishes le result to
    some "to"-topic. "from_topics_and_types" takes a list of tuples of
    "from"-topic name and its type. "to_topic_and_type" takes a tuple
    of "to"-topic name and its type. "converter" is a function that
    takes len(from_topics_and_types) number of arguments, annotated
    with corresponding "from"-topic's types, and returns a variable
    with "to"-topic type.'''


    def __init__(
            self,
            from_topics_and_types: typing.List[typing.Tuple[str, type]],
            to_topic_and_type: typing.Tuple[str, type],
            converter: typing.Callable[..., typing.Any],
            period: rospy.Duration = rospy.Duration(1)
        ) -> None:

        self.__from_topics_and_types = from_topics_and_types
        self.__to_topic_and_type = to_topic_and_type
        self.__converter: typing.Callable[..., to_topic_and_type] = converter
        self.__period = period

        funSig = inspect.signature(self.__converter)
        self.__converterTypesIn: typing.Tuple[type, ...] = typing.get_args(
            next(iter(funSig.parameters.items()))[1].annotation)
        self.__converterTypeOut: type = funSig.return_annotation

        if len(self.__from_topics_and_types) != len(self.__converterTypesIn):
            raise Exception(
                f'Numbers of ROS "from"-topics and converter function '
                f'arguments are are equal: {len(self.__from_topics_and_types)} '
                f'and {len(self.__converterTypesIn)}.')

        if list(zip(*from_topics_and_types))[1] != self.__converterTypesIn:
            raise Exception(
                f'Passed "from"-topics types differ with passed converter '
                f'function arguments annotations:\n'
                f'{str(tuple(zip(*from_topics_and_types))[1])}\n'
                f'{str(self.__converterTypesIn)}')

        if self.__converterTypeOut is inspect._empty:
            raise Exception(
                'Passed converter function does not have return annotation.')

        if self.__to_topic_and_type[1] != self.__converterTypeOut:
            raise Exception(
                f'Passed "to"-topic type differ with passed converter '
                f'function return variable annotation.')

        return


    class ClassCbTopicFrom:
        '''Class-wrapper for actual callback function. Takes topic_name that
        is supposed to be callbacked, and dictionary that is a dict
        with keys as names of topics and values as last callbacked
        value from a topic.'''


        def __init__(self, topic_name: str, dictionary: typing.Dict):

            self.__t_name: str = topic_name
            self.__dict: typing.Dict = dictionary


        def get_cb(self):
            '''Create callback function and return it.'''

            def cb_topic_from(msg):
                '''Callback to use on "from" topic.'''

                self.__dict[self.__t_name] = msg
                return

            return cb_topic_from


    def start(self):
        '''Subscribe to "from"-topics, start publishing things to
        "to"-topic.'''

        self.__t_names_and_subers = collections.OrderedDict({
            t_name: None for t_name, _ in self.__from_topics_and_types})
        self.__t_names_and_lst_vals = collections.OrderedDict({
            t_name: None for t_name, _ in self.__from_topics_and_types})

        # Create callbacks for all topics.
        for t_name, t_type in self.__from_topics_and_types:
            klass = self.ClassCbTopicFrom(t_name, self.__t_names_and_lst_vals)
            self.__t_names_and_subers[t_name] = klass.get_cb()

        # Subscribe created callbacks to le topics.
        for t_name, t_type in self.__from_topics_and_types:
            rospy.Subscriber(t_name, t_type, self.__t_names_and_subers[t_name])

        # Create publisher.
        puber = rospy.Publisher(*self.__to_topic_and_type, queue_size=10)

        def fun_publish(timer):
            '''Supposed to execute "converter" function on list of the latest
            values from "from"-topics and publish le result to
            "to"-topic.'''

            # TODO IMPORTANT! DATARACE! Create mutex for
            # self.__t_names_and_lst_vals and ClassCbTopicFrom.__dict !!!

            in_topics = self.__t_names_and_lst_vals.keys()
            in_vals = self.__t_names_and_lst_vals.values()

            silent_topics = [
                t for t, v in zip(in_topics, in_vals) if v is None]

            if not silent_topics:
                puber.publish(self.__converter(tuple(in_vals)))
                return

            print(f'topic_cvger will not post anything to topic '
                  f'"{self.__to_topic_and_type[0]}" because topic_cvger has '
                  f'gotten no data from topics: {", ".join(silent_topics)}')
            return

        
        rospy.Timer(self.__period, fun_publish)
        return
