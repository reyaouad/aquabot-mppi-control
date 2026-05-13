from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    with sl.group(ns = 'aquabot'):
        sl.node('slider_publisher',
            arguments = [sl.find('aquabot_motion', 'Twist.yaml')])
        sl.node('aquabot_motion', 'cmd.py')

        sl.node('aquabot_motion', 'planner.py')

    return sl.launch_description()
