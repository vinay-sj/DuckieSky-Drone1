#!/usr/bin/env python
from student_pid_class import PID
import sys
sys.path.insert(0, '/Users/ithier/Documents/CS5335/pidrone_pkg/scripts')
from pid_controller import PIDController, main
import yaml


class StudentPIDController(PIDController):
    """
    This is the PID controller that you will use to fly your PID class on your drone!
    Do not modify any of the code in this file.
    """

    def __init__(self):
        super(StudentPIDController, self).__init__()
        with open("z_pid.yaml", "r") as stream:
            try:
                yaml_data = yaml.safe_load(stream)
                kp, ki, kd, k = yaml_data['Kp'], yaml_data['Ki'], yaml_data['Kd'], yaml_data['K']
            except yaml.YAMLError as exc:
                print exc
                print 'Failed to load PID terms! Exiting.'
                sys.exit(1)
        self.pid.throttle = PID(kp, ki, kd, k)


if __name__ == '__main__':
    main(StudentPIDController)
