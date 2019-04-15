import bpy
import os
import errno


def main():

    # user settings
    MAX_FRAMES = 130
    outputCase = 'config_simulation_template_2_25_Josh_ToF_Calibration_tube1-23_02_54_2019_02_28'
    SIMULATION_DIR = 'D:\\Google Drive\\Undergrad\\VANTAGE\\13 Simulation'

    MM_2_M = 0.001

    bpy.context.scene.unit_settings.system = 'METRIC'

    # If the scanner is the default camera it can be accessed
    # for example by bpy.data.objects["Camera"]
    scanner = bpy.data.objects["Camera"]
    scanner.select = True

    # remove it
    bpy.ops.object.delete()
    scanner = []

    # need to add a new camera for it to actually work ... idk
    bpy.ops.object.camera_add(view_align=True, enter_editmode=False,
                              location=(0, 0, 0),
                              rotation=(1.57079632679, 0, 0),
                              layers=(True, False, False, False,
                                      False, False, False, False,
                                      False, False, False, False,
                                      False, False, False, False,
                                      False, False, False, False))
    scanner = bpy.data.objects["Camera"]
    bpy.context.object.scan_type = 'tof'

    # set the current object as the camera
    bpy.context.scene.camera = scanner

    # Move the ToF Camera to the correct spot relative to the c4d frame

    # vector from the origin of TCF to the origin of VCF, expressed in blensor
    # coords
    # [mm]
    TCFToVCF = (-49.758, -11.9, -22.0)
    scanner.location = tuple(-i * MM_2_M for i in TCFToVCF)

    # rotate the camera to face the right way
    scanner.rotation_euler[0] = -1.5708
    scanner.rotation_euler[1] = 0
    scanner.rotation_euler[2] = -3.14159

    # Setting up the ToF Camera

    # x by y resolution on sensor
    bpy.context.object.tof_xres = 352
    bpy.context.object.tof_yres = 264

    # H by V FOV
    # [deg]
    bpy.context.object.tof_lens_angle_w = 59
    bpy.context.object.tof_lens_angle_h = 44

    # maximum scan distance
    # [m]
    bpy.context.object.tof_max_dist = 10

    # focal length of ToF sensor
    # [mm]
    bpy.context.object.tof_focal_length = 10

    # noise parameters
    # who fucking knows what these mean
    bpy.context.object.tof_noise_mu = 0.00
    bpy.context.object.tof_noise_sigma = 0.01

    # physical phenoma modeling
    # backfolding can cause objects to appear closer than they actually are
    bpy.context.object.tof_backfolding = True

    # save a scan with sensor noise
    bpy.context.object.add_noise_scan_mesh = True

    # save a scan without sensor noise
    bpy.context.object.add_scan_mesh = False

    # save the scan to a file
    bpy.context.object.save_scan = True

    # use the local sensor coordinate system
    # o/w use the Blensor global coordinate system
    #
    # THIS IS IMPORTANT AF - this needs to be true for the data to be exported
    # to TCF
    bpy.context.object.local_coordinates = True

    # allow light bounces based on surface reflectivity
    bpy.context.object.ref_enabled = True

    # should the PC be visiable only in the frame it was taken
    # gui option used to better visualize the objects
    bpy.context.object.show_in_frame = True

    # store the PC data in the Blensor mesh
    bpy.context.object.store_data_in_mesh = True

    # use these to control the output coordinate system relative to the ToF /
    # global coordinate system selection
    #
    # FALSE, TRUE, TRUE will yield nominally yield TCF if using sensor
    # coordinates
    bpy.context.object.inv_scan_x = False
    bpy.context.object.inv_scan_y = True
    bpy.context.object.inv_scan_z = True

    # choose the start and end frame for a scanning sweep
    bpy.context.object.scan_frame_start = 1
    bpy.context.object.scan_frame_end = MAX_FRAMES

    # build the simulation case directory and write the truth data and the
    # config file to json files
    outputDir = os.path.join(SIMULATION_DIR, '4_Simulation_Cases')
    tofDataPath = os.path.join(outputDir, outputCase, 'ToF_Data')
    outFName = os.path.join(tofDataPath, outputCase + '.pcd')

    outDirs = [outFName]

    makeDirsFromFileNames(outDirs)

    print(outDirs)
    bpy.ops.blensor.scanrange_handler(filepath=outFName)


#
# @brief      Safely creates a dir from a full file path.
#
# @param      fpath  A list of filepaths to create
#
# @return     the directory containing the file in fpath should exist
#
def makeDirsFromFileNames(fpaths):

    for fpath in fpaths:
        if not os.path.exists(os.path.dirname(fpath)):
            try:
                os.makedirs(os.path.dirname(fpath))

            # Guard against race condition
            except OSError as exc:
                if exc.errno != errno.EEXIST:
                    raise


if __name__ == '__main__':
    main()
