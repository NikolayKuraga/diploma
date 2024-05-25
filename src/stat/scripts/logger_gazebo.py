#!/usr/bin/python3


import datetime
import logging
import pathlib

import rospy
import gazebo_msgs.srv

class LoggerGazebo:
    '''Class to log things using ROS client-server infrastructure
    (currently hardly wired with gazebo/get_model_state service).'''


    def __init__(
            self,
            service_name: str = 'gazebo/get_model_state',
            service_getter: type = gazebo_msgs.srv.GetModelState,
            rate: float = 1.):
        '''service_name is a service from 'rosservice list' in bash (e.g.
        'gazebo/get_model_state'), service_getter is a function(?) to
        get things from a service (usually provided as '.srv'-file in
        a package, e.g. 'import pkg.srv', then
        'pkg.srv.GetTheThing').'''

        self.__serv_name: str = service_name
        self.__serv_getter: type = service_getter
        self.__rate: float = rate

        log_sess = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        log_sess_path = pathlib.Path.home().joinpath('Work').joinpath('log')
        log_sess_path.mkdir(exist_ok=True)

        self.__logger_drone = self.__setup_logger(
            self.__serv_name, log_sess_path.joinpath('drone.txt'))


    def __setup_logger(
            self,
            name: str,
            log_file,
            level = logging.INFO,
            format = '%(message)s',
            clear_if_exist = True):

        if clear_if_exist: open(log_file, 'w').close()

        handler = logging.FileHandler(log_file)
        formatter = logging.Formatter(format)
        handler.setFormatter(formatter)

        logger = logging.getLogger(name)
        logger.setLevel(level)
        logger.addHandler(handler)

        return logger


    def start(self):

        rospy.wait_for_service(self.__serv_name, timeout=3)
        get_model_state = rospy.ServiceProxy(
            self.__serv_name, self.__serv_getter)

        # TODO this function somehow shall be specified out of da class
        def log_drone_position(event=None) -> str:
            msg = get_model_state('iris_demo_1', '') # Idk why like this.
            self.__logger_drone.info(( # This is all a single line.
                f'{str(msg.header.stamp.secs)}.'
                f'{str(msg.header.stamp.nsecs)},'
                f'{str(msg.pose.position.x)},'
                f'{str(msg.pose.position.y)},'
                f'{str(msg.pose.position.z)}'))
            return

        rospy.Timer(rospy.Duration(self.__rate), log_drone_position)


if __name__ == '__main__':

    rospy.init_node('logger_gazebo_node', anonymous=True)

    logger = LoggerGazebo()
    logger.start()
    
    while not rospy.is_shutdown():
        rospy.spin()
