# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/ReconstructionSystem/sensors/azure_kinect_mkv_reader.py

import argparse
import open3d as o3d
import os
import json
import sys

pwd = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(pwd, '..'))
from initialize_config import initialize_config


class ReaderWithCallback:

    def __init__(self, input, output):
        self.flag_exit = False
        self.flag_play = True
        self.input = input
        self.output = output

        self.reader = o3d.io.AzureKinectMKVReader()
        self.reader.open(self.input)
        if not self.reader.is_opened():
            raise RuntimeError("Unable to open file {}".format(args.input))

    def escape_callback(self, vis):
        self.flag_exit = True
        return False

    def space_callback(self, vis):
        if self.flag_play:
            print('Playback paused, press [SPACE] to continue.')
        else:
            print('Playback resumed, press [SPACE] to pause.')
        self.flag_play = not self.flag_play
        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_space = 32
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_space, self.space_callback)

        vis_geometry_added = False
        vis.create_window('reader', 1920, 540)

        print(
            "MKV reader initialized. Press [SPACE] to pause/start, [ESC] to exit."
        )

        if self.output is not None:
            abspath = os.path.abspath(self.output)
            metadata = self.reader.get_metadata()
            o3d.io.write_azure_kinect_mkv_metadata(
                '{}/intrinsic.json'.format(abspath), metadata)

            config = {
                'path_dataset': abspath,
                'path_intrinsic': '{}/intrinsic.json'.format(abspath)
            }
            initialize_config(config)
            with open('{}/config.json'.format(abspath), 'w') as f:
                json.dump(config, f, indent=4)

        idx = 0
        imufile = open('{}/imu.txt'.format(self.output), 'w')
        timestampfile = open('{}/frame_timestamp.txt'.format(self.output), 'w')
        imu_recorded_timestamp = []
        while True:
            imu = self.reader.next_imu()
            if imu['acc_sample_x'] != 0.0 or imu['acc_sample_y'] != 0.0 or imu['acc_sample_z'] != 0.0:
                if imu['acc_timestamp_usec'] not in imu_recorded_timestamp:
                    imufile.write(
                        str(int(imu['acc_timestamp_usec'])) + '\t' + str(imu['acc_sample_x']) + '\t' + str(
                            imu['acc_sample_y']) + '\t' + str(imu['acc_sample_z']) + '\t' + str(
                            int(imu['gyro_timestamp_usec'])) + '\t' +
                        str(imu['gyro_sample_x']) + '\t' + str(imu['gyro_sample_y']) + '\t' + str(
                            imu['gyro_sample_z']) + '\n')
                    imu_recorded_timestamp.append(imu['acc_timestamp_usec'])
            else:
                break
        while not self.reader.is_eof() and not self.flag_exit:
            if self.flag_play:
                frame_result = self.reader.next_frame_with_timestamp()
                rgbd = frame_result[0]
                if rgbd is None:
                    continue
                timestamp = frame_result[1]
                timestampfile.write(str(timestamp) + '\n')
                if not vis_geometry_added:
                    vis.add_geometry(rgbd)
                    vis_geometry_added = True

                if self.output is not None:
                    color_filename = '{0}/color/{1:05d}.jpg'.format(
                        self.output, idx)
                    print('Writing to {}'.format(color_filename))
                    o3d.io.write_image(color_filename, rgbd.color)

                    depth_filename = '{0}/depth/{1:05d}.png'.format(
                        self.output, idx)
                    print('Writing to {}'.format(depth_filename))
                    o3d.io.write_image(depth_filename, rgbd.depth)
                    idx += 1

            try:
                vis.update_geometry(rgbd)
            except NameError:
                pass
            vis.poll_events()
            vis.update_renderer()

        self.reader.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv reader.')
    parser.add_argument('--input',
                        type=str,
                        required=True,
                        help='input mkv file')
    parser.add_argument('--output',
                        type=str,
                        help='output path to store color/ and depth/ images')
    args = parser.parse_args()

    if args.input is None:
        parser.print_help()
        exit()

    if args.output is None:
        print('No output path, only play mkv')
    elif os.path.isdir(args.output):
        print('Output path {} already existing, only play mkv'.format(
            args.output))
        args.output = None
    else:
        try:
            os.mkdir(args.output)
            os.mkdir('{}/color'.format(args.output))
            os.mkdir('{}/depth'.format(args.output))
        except (PermissionError, FileExistsError):
            print('Unable to mkdir {}, only play mkv'.format(args.output))
            args.output = None

    reader = ReaderWithCallback(args.input, args.output)
    reader.run()
