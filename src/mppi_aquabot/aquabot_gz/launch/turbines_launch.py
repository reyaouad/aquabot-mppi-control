from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('world', 'easy')
sl.declare_arg('gui', True)


def launch_setup():

    # launch world
    world = sl.arg('world')
    if 'aquabot' not in world:
        world = f'aquabot_windturbines_{world}'
    world.strip('.sdf')

    gz_args = '-r'
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(sl.find('aquabot_gz', world+'.sdf'), gz_args=gz_args)

    bridges = [GazeboBridge.clock()]

    # turbine bridges
    bridges.append(('/vrx/windturbinesinspection/windturbine_checkup',
                    '/vrx/windturbinesinspection/windturbine_checkup',
                    'std_msgs/msg/String', GazeboBridge.ros2gz))
    bridges.append(('/aquabot/ais_sensor/windturbines_positions',
                    '/aquabot/ais_sensor/windturbines_positions',
                    'geometry_msgs/msg/Pose', GazeboBridge.gz2ros))
    sl.create_gz_bridge(bridges)

    # spawn aquabot robot
    with sl.group(ns = 'aquabot'):

        sl.robot_state_publisher('aquabot_description', 'aquabot.urdf')
        sl.spawn_gz_model('aquabot')

        # add bridges for this model
        bridges = []
        bridges.append((f'/world/{world}/model/aquabot/joint_state',
                        'joint_states',
                        'sensor_msgs/msg/JointState', GazeboBridge.gz2ros))
        # command
        for side in ('left','right'):
            bridges.append((f'/aquabot/thrusters/{side}/pos',
                            f'thrusters/{side}/cmd_pos',
                            'std_msgs/Float64', GazeboBridge.ros2gz))
            bridges.append((f'/aquabot/thrusters/{side}/thrust',
                            f'thrusters/{side}/thrust',
                            'std_msgs/Float64', GazeboBridge.ros2gz))
        bridges.append(('/aquabot/thrusters/main_camera_sensor/pos',
                        'camera/cmd_pos',
                        'std_msgs/Float64', GazeboBridge.ros2gz))

        # sensors
        bridges.append((f'/world/{world}/model/aquabot/link/aquabot/gps_link/sensor/navsat/navsat',
                        'sensors/gps/gps/fix',
                        'sensor_msgs/NavSatFix', GazeboBridge.gz2ros))
        bridges.append((f'/world/{world}/model/aquabot/link/aquabot/imu_link/sensor/imu_sensor/imu',
                        'sensors/imu/imu/data',
                        'sensor_msgs/Imu', GazeboBridge.gz2ros))
        bridges.append((f'/world/{world}/model/aquabot/link/aquabot/main_camera_post_link/sensor/main_camera_sensor/image',
                        'sensors/cameras/main_camera_sensor/image_raw',
                        'sensor_msgs/Image', GazeboBridge.gz2ros))
        for info in ('range', 'bearing'):
            bridges.append((f'/aquabot/sensors/acoustics/receiver/{info}',
                            f'sensors/acoustics/receiver/{info}',
                            'std_msgs/Float64', GazeboBridge.gz2ros))

        sl.create_gz_bridge(bridges)

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
