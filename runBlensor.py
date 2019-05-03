import bpy
import os
import errno

__author__ = "Nicholas Renninger"
__copyright__ = "'Copyright' 2019, VANTAGE"
__credits__ = ["Long Song Silver"]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Nicholas Renninger"
__email__ = "nicholas.renninger@colorado.edu"
__status__ = "Development"


def configureRunSaveBlensorScanRange():

    veryImportantFunction()

    ###############
    # user settings
    ###############

    # ability to turn off scanning if you just want to verify the output
    # directory structure before running simulation batch
    SHOULD_RUN_SCANS = True

    # run scans from frame 1 to MAX_FRAMES
    MAX_FRAMES = 45

    # because blensor uses a dispatch architecture with no easy observer
    # pattern implementation, you can't programmatically know when a scan is
    # done. Thus, to run multiple scans for the same file, you run this script
    # with a number of times to dispatch the scanner
    #
    # Changing NUM_SIMS_TO_RUN changes the maximum value of X in the 'SampleX'
    # directory within the current case's ToF Data folder.
    #
    # e.g. NUM_SIMS_TO_RUN = 4 will create the following folders in the
    # outputCase directory:
    # PATH_TO_SIMULATION_DIR/4_Simulation_Cases/$outputCase/ToF_Data/Sample1
    # PATH_TO_SIMULATION_DIR/4_Simulation_Cases/$outputCase/ToF_Data/Sample2
    # PATH_TO_SIMULATION_DIR/4_Simulation_Cases/$outputCase/ToF_Data/Sample3
    # PATH_TO_SIMULATION_DIR/4_Simulation_Cases/$outputCase/ToF_Data/Sample4
    # with all of the pcd files for each scan in each "sample" dir
    #
    # Change this number to easily create new folders containing successive
    # scans of the same case
    NUM_SIMS_TO_RUN = 10
    START_SAMPLE_NUM = 1
    outputCase = 'VTube4_DTube3_CubeSatsSix1U_Speed250cmps-16_15_59_2019_04_15'

    # sensor Gaussian process noise
    # adjust these to make the ToF returns look more like real sensor data

    # nominal value: 0
    noise_mu = 0

    # nominal value: [0.01 - 0.02]
    noise_sigma = 0.01

    ###########################################################################
    # ABS PATH to the directory containing the simulation repo
    # directories will look like the following:
    # */<SIMULATION_DIR>/4_Simulation_Cases/
    # */<SIMULATION_DIR>/VANTAGE_Simulation/
    #
    # Thus, set SIMULATION_DIR such that it contains the absolute path to the
    # overall <SIMULATION_DIR> shown above.
    #
    # Nick Workstation
    # SIMULATION_DIR = 'F:\\Cloud\\Google Drive\\Undergrad\\VANTAGE\\13 Simulation'

    # Nick LT
    SIMULATION_DIR = 'D:\\Google Drive\\Undergrad\\VANTAGE\\13 Simulation'
    ###########################################################################

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
    bpy.context.object.tof_noise_mu = noise_mu
    bpy.context.object.tof_noise_sigma = noise_sigma

    # physical phenomena modeling
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

    # should the PC be visible only in the frame it was taken
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

    for caseNum in range(START_SAMPLE_NUM, NUM_SIMS_TO_RUN + 1):

        # create directory for each time the "sample" case is run
        sampleStr = 'Sample' + str(caseNum)
        tofDataPath = os.path.join(outputDir, outputCase, 'ToF_Data',
                                   sampleStr)
        outFName = os.path.join(tofDataPath, sampleStr + '_' +
                                outputCase + '.pcd')

        makeDirsFromFileNames([outFName])

        # simulates pushing the 'scan range' button in the Blensor sensor GUI
        # to ensure everything is set properly
        print('-------------------------------------------------------------')
        print('Running Simulation Number ', caseNum)
        if SHOULD_RUN_SCANS:
            bpy.ops.blensor.scanrange_handler(filepath=outFName)
        print('Simulation finished with output to:\n', tofDataPath)
        print('-------------------------------------------------------------')


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


#
# @brief      does all of the things
#
# @return     never change this or we'll all die
#
def veryImportantFunction():

    # super important NEVER CHANGE THIS it will break all of the coding and
    # algorithms
    print('\n\n\n\n\n\n\n\n\n=================================')
    print('\nSwaqeroni mAceroni')
    print('\n=================================')


if __name__ == '__main__':

    configureRunSaveBlensorScanRange()
