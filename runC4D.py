import c4d
import yaml
import os
import json
import errno
import shutil
import datetime

__author__ = "Nicholas Renninger, Jerry Wang"
__copyright__ = "'Copyright' 2019, VANTAGE"
__credits__ = ["Long Song Silver"]
__license__ = "MIT"
__version__ = "0.0.2"
__maintainer__ = "Nicholas Renninger"
__email__ = "nicholas.renninger@colorado.edu"
__status__ = "Development"


def main():

    # clear all simulation objects
    # Select All
    c4d.CallCommand(100004766)
    # Delete
    c4d.CallCommand(100004787)
    # Clear Console
    c4d.CallCommand(13957, 13957)

    (settingsFile, configFileName, configSetupFile) = getConfigFile()
    runAndSaveSimulation(settingsFile, configFileName, configSetupFile)


#
# @brief      Gets the configuration file from the static file in the curr dir
#
# @return     The configuration filename WITH and WITHOUT the extension, and
#             the name of the file where you set which config file to run
#             @tuple of @str
#
def getConfigFile():

    configSetupFile = 'currSimConfigFile'

    # need to ensure that the cwd of the script is the same directory as this
    # python file so it can find the configuration configSetupFile
    os.chdir(os.path.dirname(os.path.realpath(__file__)))

    try:
        with open(configSetupFile, 'r') as stream:
            configFileName = file.read(stream)

    except IOError as err:
        print 'a file named ' + configSetupFile +\
            '\nwith the name of the desired config file\n' +\
            'should be in the same dir as this python script\n\n'
        raise err

    settingsFile = os.path.join('config',
                                configFileName + '.yaml')
    print 'Using ' + settingsFile
    return (settingsFile, configFileName, configSetupFile)


#
# @brief      Creates an entire simulation case given the parameters in the
#             settingsFile
#
# @param      settingsFile     The full path to the YAML settings file
# @param      configFileName   The configuration file name
# @param      configSetupFile  The configuration setup file where you choose
#                              the config file to run
#
# @return     a saved C4D simulation case in the C4D case simulation directory
#
def runAndSaveSimulation(settingsFile, configFileName, configSetupFile):

    # this is the current c4d session we need to modify
    doc = c4d.documents.GetActiveDocument()

    # all settings are defined in settingsFile
    settings = simulationSetup(settingsFile, configSetupFile, doc)
    settings['CONFIG_NAME'] = configFileName

    # load in the deployer model and set it up
    des_tube_origin = setup_deployer(settings, doc)[1]

    # load in the all of the cubesats, position them, and prescribe their
    # motion
    cubeSats = setup_cubesats(settings, des_tube_origin, doc)

    # add the camera after the VANTAGE origin has already been set
    addCamera(settings, doc)

    # saving the actual position and orientation of the CubeSats at each time
    # step
    saveCubeSatStates(settings, doc, cubeSats)

    # save current path to restore after saving files this is done to allow you
    # to re-run script quickly without having to manually set the scripting dir
    # path
    curr_path = os.getcwd()

    # Save C4D file
    c4d.CallCommand(12218, 12218)

    # Export as fbx
    c4d.CallCommand(60000, 10)

    # go back to the old path
    os.chdir(curr_path)

    print 'Done with loading in simulation case. Have Fun!!'


#
# @brief      Loads in YAML config file settings and sets all overall
#             simulation settings from YAML config
#
#             Example settings: camera FPS, total simulation time, lighting,
#             etc.
#
# @param      settingsFile     The settings YAML relative filepath
# @param      configSetupFile  The configuration setup file where you choose
#                              the config file to run
# @param      doc              current C4D document to setup simulation in
#
# @return     Settings dict from YAML config
#
def simulationSetup(settingsFile, configSetupFile, doc):

    try:
        # Read YAML settings file
        with open(settingsFile, 'r') as stream:
            settings = yaml.load(stream)

    except IOError as err:
        print configSetupFile + ' likely does not point to a valid \n' +\
            'configuration file in the ../config/ directory:\n\n'
        raise(err)

    settings['CONFIG_PATH'] = settingsFile

    # ====================================
    # set the overall simulation parameters
    # ====================================
    fps = settings['TOF_FPS']
    doc.SetFps(fps)

    # Cinema 4D has a time class that internally stores the time values as
    # exact fractions independent of the frame rate.
    #
    # We need to calculate the maximum simulation time from the min z-linear
    # velocities and the maximum z-dist of the simulation
    maxRng = settings['MAX_RANGE']
    ZVels = settings['CUBESAT_LIN_VELS']['v_z']
    maxTimeInS = maxRng / min(ZVels)
    maxTimeInC4DBaseTime = c4d.BaseTime(maxTimeInS)
    doc.SetMaxTime(maxTimeInC4DBaseTime)

    # keep the max simulation time / frame for later
    settings['MAX_TIME'] = maxTimeInS
    settings['MAX_FRAME'] = maxTimeInC4DBaseTime.GetFrame(doc.GetFps())

    print 'Done Configuring Simulation'

    return settings


#
# @brief      Adds the camera and updates the rendering settings to use the new
#             camera settings
#
# @param      settings  The settings dict from the config file
# @param      doc       The current c4d document
#
# @return     the active camera should be set to a placed properly in the
#             deployer, with all of the camera properties given in the config
#             file
#
def addCamera(settings, doc):

    camera = c4d.BaseObject(c4d.Ocamera)
    rdata = doc.GetActiveRenderData()

    foc_length = settings['FOCAL_LENGTH']
    Ape = settings['APERTURE']
    shutterSpeed = settings['SHUTTER_SPEED']
    xres = settings['X_Res']
    yres = settings['Y_Res']
    fps = settings['OPTICAL_CAMERA_FPS']
    cameraloc = settings['CAMERA_LOC']
    camerarot = settings['CAMERA_ROT']
    fStop = settings['F_STOP']

    # setting camera properties
    camera()[c4d.CAMERA_FOCUS] = foc_length
    camera()[c4d.CAMERAOBJECT_APERTURE] = Ape
    camera()[c4d.CAMERAOBJECT_SHUTTER_SPEED_VALUE] = shutterSpeed
    camera()[c4d.CAMERAOBJECT_FNUMBER_VALUE] = fStop

    # setting camera location and orientation relative to the VANTAGE frame
    camera()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_X] = cameraloc[0]
    camera()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_Y] = cameraloc[1]
    camera()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_Z] = cameraloc[2]
    camera()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_X] = camerarot[0]
    camera()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_Y] = camerarot[1]
    camera()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_Z] = camerarot[2]

    # setting render settings
    rdata[c4d.RDATA_XRES] = xres
    rdata[c4d.RDATA_YRES] = yres
    rdata[c4d.RDATA_FRAMERATE] = fps

    # turn on physics rendering engine to set the majority of the parameters
    rdata[c4d.RDATA_RENDERENGINE] = 0

    # changing to PNG sequence output
    rdata[c4d.RDATA_FORMAT] = 1023671

    # changing to rendering the full frame
    rdata[c4d.RDATA_FRAMESEQUENCE] = 2

    # insert the configured camera object
    doc.InsertObject(camera)

    # changing the current render viewpoint to the inserted camera
    c4d.CallCommand(55000, 2)

    # need to swap to the ProRender engine, but C4D has a bug that makes it so
    # that it doesn't immediately switch. Thus, we toggle back to ProRender,
    # back to Physical, and again back to ProRender to hopefully switch to the
    # ProRender engine.
    #
    # NOT CURRENTLY WORKING in C4D R20.028.
    rdata[c4d.RDATA_RENDERENGINE] = 1037639
    rdata[c4d.RDATA_RENDERENGINE] = 0
    rdata[c4d.RDATA_RENDERENGINE] = 1037639


#
# @brief      Loads in the deployer model and positions it such that the global
#             origin is aligned with the VANTAGE Cartesian frame origin
#
# @param      settings  Settings dict from YAML config
# @param      doc       current C4D document to add deployer to
#
# @return   IDX:   Description
#             0:   a reference to the deployer C4D object
#             1:   a list containing the desired deployer tube's origin in
#                  x,y,z coords in the global simulation coord system
#                  [x, y, z] ~ [cm]
#
def setup_deployer(settings, doc):

    dep = settings['DEPLOYER_FILEPATH']
    simLoc = settings['SIM_DIR_LOCATION']
    fullFile = os.path.join(simLoc, dep)

    # Import Deployer model
    c4d.documents.MergeDocument(doc, fullFile, c4d.SCENEFILTER_OBJECTS |
                                c4d.SCENEFILTER_MATERIALS)
    deployerObj = doc.GetActiveObject()

    # Position and Rotate the Deployer so the global origin is at VANTAGE's
    # origin
    #
    # After moving the deployer origin
    # to the correct position in the global coordinate frame, then move the
    # deployer origin from the centerline to the VCF origin according to these
    # parameters
    #
    # This location is just a tube number (1-6), so need to use geometry
    # defined in 13-5 to turn into global coords
    VANTAGE_tube_num = settings['VANTAGE_POS']
    launch_tube_num = settings['DESIRED_DEPLOYMENT_TUBE']

    if launch_tube_num == VANTAGE_tube_num:
        msg = 'You can''t launch (tube ' +\
              str(launch_tube_num) +\
              ') from the same tube as VANTAGE (tube ' +\
              str(VANTAGE_tube_num) + ')'
        raise ValueError(msg)

    rho = settings['RHO']
    nabla = settings['NABLA']

    # convert focal length from [mm] to [cm]
    f = settings['FOCAL_LENGTH'] / 10.
    p_optical = settings['P_OPTICAL']

    # need to figure out where the VCF origin is relative to the deployer
    # origin
    # [cm]
    d_optical = p_optical - f
    z_loc_of_VCF = d_optical

    desired_tube = VANTAGE_tube_num
    translVec = [nabla, rho, z_loc_of_VCF]
    VANTAGE_origin_pos = calculateOriginFromTubeNumber(desired_tube,
                                                       settings, translVec)

    # solidworks models have x and y axes flipped, so correct this
    VANTAGE_origin_pos[0] *= -1
    VANTAGE_origin_pos[1] *= -1
    VANTAGE_origin_rot = settings['VANTAGE_ROT']
    pos_rot_obj(deployerObj, VANTAGE_origin_pos, VANTAGE_origin_rot)

    c4d.EventAdd()

    # define the deployment tube origin

    # need to locate tube origin in global coords by first calculating the
    # relative tube origin then adding to this relative position the location
    # of the deployer's origin in global coords
    # [cm]
    translVec = VANTAGE_origin_pos
    desired_tube = launch_tube_num
    des_tube_origin = calculateOriginFromTubeNumber(desired_tube,
                                                    settings, translVec)

    # Kobe
    print "Loaded Deployer Model"

    return deployerObj, des_tube_origin


#
# @brief      Calculates the tube origin given the desired tube and the
#
# @param      desired_tube  The desired tube
# @param      settings      Settings dict from YAML config
# @param      translVec     The translation vector to move the tube origin to
#                           the origin of the final desired frame
#
# @return     The tube origin vector in global coords [cm]
#
def calculateOriginFromTubeNumber(desired_tube, settings, translVec):

    # save deployer geometry variables
    mu = settings['MU']
    psi = settings['PSI']
    gamma = settings['GAMMA']
    L = settings['L']

    tube_coeffs = settings['TUBE_ORIGIN_COEFFS'][desired_tube - 1]
    x_coeff = tube_coeffs['x']
    y_coeff = tube_coeffs['y']
    z_coeff = tube_coeffs['z']

    des_tube_x = (x_coeff[0] * psi + 0.5 * x_coeff[1] * gamma)
    des_tube_y = y_coeff[0] * mu
    des_tube_z = (0.5 * L) * z_coeff[0]

    # now, translate this tube origin to its correct place in the global
    # coordinate system by adding the translation that moves the tube origin by
    # the desired final translation vector
    des_tube_x += translVec[0]
    des_tube_y += translVec[1]
    des_tube_z += translVec[2]

    # package output
    return [des_tube_x, des_tube_y, des_tube_z]


#
# @brief      Adds all cubesats to the current simulation, places them in the
#             desired tube in the right order, and prescribes their motion
#
# @warning    assumes the deployer tube origin is given at the planar center at
#             the back of the tube and that the cubesat origin in C4D is at the
#             CubeSat's origin
#
# @param      settings         Settings dict from YAML config
# @param      des_tube_origin  a list containing the desired deployer tube's
#                              origin in x,y,z coords in the global simulation
#                              coord system
#                              [x, y, z] ~ [cm]
# @param      doc              current C4D document to add cubesats to
#
# @return     a list of each C4D cubesat object added to the simulation
#
def setup_cubesats(settings, des_tube_origin, doc):
    objs = []
    simLoc = settings['SIM_DIR_LOCATION']

    cubeSatSizes_strs = settings['CUBESATS_SIZES']
    cubeSatSizes_conversion = settings['U_STR_TO_U']

    # [cm]
    u_size = settings['U_SIZE_DEF']

    # the length of the little tabs on either end of a cubesat
    # [cm]
    cubeSat_tab_size = settings['CUBESAT_TAB_LENGTH']

    # initialize the back wall to be the back of the deployer tube
    # [cm]
    curr_back_wall = des_tube_origin[2]

    # The center of each CubeSat will be along the boresight of the deployer
    # tube
    # [cm]
    pos = [des_tube_origin[0], des_tube_origin[1], curr_back_wall]

    # shouldn't need to rotate the cubeSats
    # [rad]
    rot = [0, 0, 0]

    # the cubesat should start flush with the back of the deployer tube
    # [cm]
    currSep = 0

    n = len(cubeSatSizes_strs)

    for ii in range(0, n):

        # angular and linear velocity vectors of Cubesat
        # [cm/s]
        v_x = settings['CUBESAT_LIN_VELS']['v_x'][ii]
        v_y = settings['CUBESAT_LIN_VELS']['v_y'][ii]
        v_z = settings['CUBESAT_LIN_VELS']['v_z'][ii]
        v = [v_x, v_y, v_z]

        # [rad/s]
        omega_x = settings['CUBESAT_ROT_VELS']['omega_x'][ii]
        omega_y = settings['CUBESAT_ROT_VELS']['omega_y'][ii]
        omega_z = settings['CUBESAT_ROT_VELS']['omega_z'][ii]
        omega = [omega_x, omega_y, omega_z]

        # get a number (e.g. 1, 2, 3, etc.) corresponding to a size of ('1U',
        # '2U', '3U', etc.)
        # [U]
        CubeSatSize = cubeSatSizes_conversion[cubeSatSizes_strs[ii]]

        # Import the CubeSat Model
        cubesat = settings['CUBESAT_FILEPATHS'][cubeSatSizes_strs[ii]]
        fullFile = os.path.join(simLoc, cubesat)

        # get length of the CubeSat Model
        # [cm]
        cubesat_length = CubeSatSize * u_size + 2 * cubeSat_tab_size

        c4d.documents.MergeDocument(doc, fullFile, 1)
        cubeSatObj = doc.GetActiveObject()

        # Position the z-location of the CubeSat's origin (position in the
        # length of the tube)
        # [cm]
        pos[2] = curr_back_wall + cubesat_length / 2

        # update the separation between the current and next cubesat
        if ii < n - 1:
            currSep = settings['CUBESAT_SEPARATIONS'][ii]

        # Move the "back wall" to be the front of the previous cubesat, plus
        # any separation bewtween each cubesat
        curr_back_wall += cubesat_length + currSep

        # need to define all animation inputs
        posStart = pos[:]
        rotStart = rot[:]

        print(posStart)

        # move cubesat to the right location and animate it's motion
        # (translation and rotation) over the duration of the simulation
        cubeSatObj = animateCubeSat(cubeSatObj, posStart, rotStart,
                                    v, omega, cubesat_length,
                                    settings, des_tube_origin, doc)

        objs.append(cubeSatObj)

        print 'Loaded in a ' + cubeSatSizes_strs[ii] + ' CubeSat Model'

    return objs


#
# @brief      moves and rotates the C4D obj by pos and rot respectively
#
# @param      obj   The C4D object to manipulate
# @param      pos   A list of positions in global coords to move obj by [cm]
# @param      rot   A list of euler angles rotations to rotate obj by [obj]
#
# @return     obj has been moved and rotated to pos and rot, respectively
#
def pos_rot_obj(obj, pos, rot):
    obj()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_X] = rot[0]
    obj()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_Y] = rot[1]
    obj()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_Z] = rot[2]
    obj()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_X] = pos[0]
    obj()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_Y] = pos[1]
    obj()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_Z] = pos[2]


#
# @brief      Positions and animates the CubeSat object based on the
#             simulation, animation, and geometric definitions given in the
#             config file
#
#             @see getAnimationFrameNumbers and @see getLastPosAndRot for most
#             of the specific animation calculations - animations are applied
#             with the deployer movement constraints in mind
#
# @param      cubeSatObj       The cubesat object to place and animate
# @param      posStart         The start location of the CubeSat's centroid
#                              in global coords.
#                              [cm, cm, cm]
# @param      rotStart         The start rotation of the CubeSat
#                              in global coords.
#                              [rad, rad, rad]
# @param      cubeSatLinVel    The cube sat lin velocity vector to animate
#                              [cm/s, cm/s, cm/s]
# @param      cubeSatRotVel    The cube sat rot velocity vector to animate
#                              [rad/s, rad/s, rad/s]
# @param      cubesat_length   The total cubesat length
#                              [cm]
# @param      settings         The settings dict containing all config
#                              parameters
# @param      des_tube_origin  The desired deployment tube origin in global
#                              coords
#                              [cm]
# @param      doc              The current C4D document to add the cubesat
#                              animation to
#
# @return     cubeSatObj will have realistic animations of the desired linear
#             and rotational velocities
#
def animateCubeSat(cubeSatObj, posStart, rotStart,
                   cubeSatLinVel, cubeSatRotVel, cubesat_length, settings,
                   des_tube_origin, doc):
    # [cm]
    L = settings['L']

    # time until the cubesats reach MAX_DISTANCE
    # [s]
    max_time = settings['MAX_TIME']

    fps = doc.GetFps()

    # getting the starting and ending frames for all of the animations
    (linVelZStartFrame,
     rotAndLinXYVelStartFrame,
     endFrame) = getAnimationFrameNumbers(L, cubesat_length,
                                          posStart, max_time,
                                          des_tube_origin,
                                          fps, cubeSatLinVel[2])

    # get the ending position and velocity of the CubeSats
    posEnd, rotEnd = getLastPosAndRot(max_time, fps, cubeSatLinVel,
                                      cubeSatRotVel, L, posStart,
                                      rotAndLinXYVelStartFrame, endFrame)

    # get animation tracks for position and rotation
    obj_type_id = c4d.ID_BASEOBJECT_POSITION
    xtrack = getXYZtrack(cubeSatObj, obj_type_id, c4d.VECTOR_X, doc)
    ytrack = getXYZtrack(cubeSatObj, obj_type_id, c4d.VECTOR_Y, doc)
    ztrack = getXYZtrack(cubeSatObj, obj_type_id, c4d.VECTOR_Z, doc)

    obj_type_id = c4d.ID_BASEOBJECT_ROTATION
    wxtrack = getXYZtrack(cubeSatObj, obj_type_id, c4d.VECTOR_X, doc)
    wytrack = getXYZtrack(cubeSatObj, obj_type_id, c4d.VECTOR_Y, doc)
    wztrack = getXYZtrack(cubeSatObj, obj_type_id, c4d.VECTOR_Z, doc)

    # start recording addition of animations to timeline
    cubeSatObj.InsertTrackSorted(xtrack)
    cubeSatObj.InsertTrackSorted(ytrack)
    cubeSatObj.InsertTrackSorted(ztrack)

    # add position animation - linear variation of position from start to end
    # frames
    addValueAtFrame(xtrack, ytrack, ztrack, rotAndLinXYVelStartFrame,
                    linVelZStartFrame, fps, posStart)
    addValueAtFrame(xtrack, ytrack, ztrack, endFrame, endFrame, fps, posEnd)

    # add rotation animation - linear variation of rotation from start to end
    # frames
    addValueAtFrame(wxtrack, wytrack, wztrack, rotAndLinXYVelStartFrame,
                    rotAndLinXYVelStartFrame, fps, rotStart)
    addValueAtFrame(wxtrack, wytrack, wztrack, endFrame, endFrame, fps, rotEnd)

    # Goto Start
    c4d.CallCommand(12501)

    return cubeSatObj


#
# @brief      Used to get an animation track for a given type of track for the
#             given channel
#
# @param      obj        The c4d object to get an animation track for
# @param      trackID    The track id (track type) - defines what sort
#                        animation you want to add to the given channel e.g.:
#
#                       \code{.py}
#                       # animate postion
#                       trackID = c4d.ID_BASEOBJECT_POSITION
#
#                       # animate postion
#                       trackID = c4d.ID_BASEOBJECT_ROTATION
#                       \endcode
#
# @param      channelID  The channel id (channel type) defines what channel
#                        you want to add the given animation type to e.g.:
#
#                       \code{.py}
#                       # use x animation channel for the animation type
#                       # (trackID)
#                       channelID = c4d.VECTOR_X
#
#                       # use y animation channel for the animation type
#                       # (trackID)
#                       channelID = c4d.VECTOR_Y
#
#                       # use z animation channel for the animation type
#                       # (trackID)
#                       channelID = c4d.VECTOR_Z
#                       \endcode
# @param      doc        The current C4D document in use
#
# @return     The xyz track with the given trackID and channelID
#
def getXYZtrack(obj, trackID, channelID, doc):
    # Find the Track
    param = c4d.DescID(c4d.DescLevel(trackID, c4d.DTYPE_VECTOR, 0),
                       c4d.DescLevel(channelID, c4d.DTYPE_REAL, 0))
    track = obj.FindCTrack(param)

    # Create if no track found
    if not track:
        track = c4d.CTrack(obj, param)
        doc.AddUndo(c4d.UNDOTYPE_NEW, track)
        obj.InsertTrackSorted(track)

    return track


#
# @brief      Adds a frame animation value at the given frame #
#
# @param      xtrack        The X-component animation track to add the value to
# @param      ytrack        The Y-component animation track to add the value to
# @param      ztrack        The Z-component animation track to add the value to
# @param      XYFrameNum    The frame # to add the XY-component value to
# @param      ZFrameNum     The frame # to add the Z-component value to
# @param      fps           The fps of the simulation
# @param      valueVec      The list of values [x, y, z] to add to each
#                           respective animation track
#
# @return     xtrack, ytrack, and ztrack have valueVec[0], valueVec[1],
#             valueVec[2] (respectively) added in the XYFrameNum and ZFrameNum
#             animation frames
#
def addValueAtFrame(xtrack, ytrack, ztrack, XYFrameNum,
                    ZFrameNum, fps, valueVec):

    # linear interpolation
    animation_interpolation_val = 465001092

    # Add a key to each CubeSat linear position tracks
    xtrack = xtrack.GetCurve()
    xkey = xtrack.AddKey(c4d.BaseTime(XYFrameNum, fps))['key']
    xkey.SetValue(xtrack, valueVec[0])

    # set the track position variation to linear
    c4d.CallCommand(animation_interpolation_val)

    ytrack = ytrack.GetCurve()
    ykey = ytrack.AddKey(c4d.BaseTime(XYFrameNum, fps))['key']
    ykey.SetValue(ytrack, valueVec[1])

    # set the track position variation to linear
    c4d.CallCommand(animation_interpolation_val)

    ztrack = ztrack.GetCurve()
    zkey = ztrack.AddKey(c4d.BaseTime(ZFrameNum, fps))['key']
    zkey.SetValue(ztrack, valueVec[2])

    # set the track position variation to linear
    c4d.CallCommand(animation_interpolation_val)


#
# @brief      Calculates what the final position and total rotational change of
#             the object should be at the last frame of the simulation.
#
#             Takes into account the simulation fps, the maximum simulation z
#             distance, and the fact that (x/y linear) / rotational velocities
#             start at different frames than the z linear velocity.
#
#             @see getAnimationFrameNumbers() for more info on how animation
#                  start / end frames are calculated
#
# @param      max_time                  The maximum simulation time [s]
# @param      fps                       The fps of the simulation [cm]
# @param      v                         The linear velocity vector of the
#                                       object [cm/s]
# @param      omega                     The rotational velocity vector of the
#                                       object [rad/s]
# @param      L                         The length of the deployer tube [cm]
# @param      posStart                  The starting position vector of the
#                                       object [cm, cm, cm]
# @param      rotAndLinXYVelStartFrame  The rot and lin xy velocity start frame
#                                       [frame #]
# @param      endFrame                  The end frame of the simulation
#                                       [frame #]
#
# @return     [final position vector of the object at the endFrame [cm],
#              total rotation of the object at the endFrame [rad]]
#
def getLastPosAndRot(max_time, fps, v, omega, L, posStart,
                     rotAndLinXYVelStartFrame, endFrame):
    # the cubesat can only start rotating / moving in X-Y after its not
    # constrained by the deployer tube
    rot_linXY_time = (endFrame - rotAndLinXYVelStartFrame) / fps

    # get the final position of the cubeSat
    # [cm]
    x_f = posStart[0] + v[0] * rot_linXY_time
    y_f = posStart[1] + v[1] * rot_linXY_time
    z_f = posStart[2] + v[2] * max_time + L

    posEnd = [x_f, y_f, z_f]

    # get the final rotation of the cubeSat
    rot_linXY_time = (endFrame - rotAndLinXYVelStartFrame) / fps

    # [rad]
    rotEnd = [rot_linXY_time * omega[0],
              rot_linXY_time * omega[1],
              rot_linXY_time * omega[2]]

    return posEnd, rotEnd


#
# @brief      Gets the animation start and stop frame numbers given simulation
#             parameters
#
#             Takes into account that the animation for x,y linear velocity and
#             all rotational velocities can only begin once the cubesat is
#             clear from the deployer tube (the tube constrains a cubesat in
#             all but the linear z direction)
#
# @param      L                Length of the Deployer [cm]
# @param      cubesat_length   The cubesat length [cm]
# @param      posStart         The starting centroid position of the cubesat
#                              [cm]
# @param      max_time         The maximum time to simulate to
# @param      des_tube_origin  The desired tube origin in global coords [cm]
# @param      fps              The fps of the simulation [frames / s]
# @param      v_z              The main cubesat velocity component [cm/s]
# @param      posEnd  The ending centroid position of the cubesat [cm]
#
# @return     [start frame for lin. z velocity animation, start frame for rot.
#             and lin. x-y velocity animation, start frame for both lin. & rot
#             velocity animation]
#
def getAnimationFrameNumbers(L, cubesat_length, posStart, max_time,
                             des_tube_origin, fps, v_z):

    # need to compute frame numbers for where to apply the animation starts and
    # stops for linear and rotational animations

    maxDist = v_z * max_time

    # we need the CubeSats to be moving linearly in z from the very first frame
    linVelZStartFrame = 0

    # defines the initial location of the CubeSat's centroid
    # [cm]
    d_o = posStart[2]

    # defines the exact distance of the CubeSat's centroid when it has cleared
    # the deployer
    # [cm]
    d_f = (cubesat_length / 2) + des_tube_origin[2] + L

    # the CubeSats can only start rotating once they are clear from the
    # deployer
    delta_z = d_f - d_o
    rotAndLinXYVelStartFrame = (delta_z / v_z) * fps

    # last frame is the same for both linear and rotational motion
    endFrame = linVelZStartFrame + ((L + maxDist) / v_z) * fps

    return linVelZStartFrame, rotAndLinXYVelStartFrame, endFrame


#
# @brief      Saves CubeSat states (position and orientation) at each timestep
#
#             This data is exported in the VANTAGE cartesian frame (VCF) as
#             defined in 13-5
#
# @param      settings  The settings dict from the config file
# @param      doc       The current c4d document
# @param      cubesats  a list of each C4D cubesat object added to the
#                       simulation
#
# @return     a json file with the states of each cubesat at each timestep
#
def saveCubeSatStates(settings, doc, cubesats):

    maxFrame = settings['MAX_FRAME']
    fps = doc.GetFps()

    cubeSatStates = []
    frames = range(0, maxFrame + 1)

    for frame in frames:

        # [s]
        currTime = frame / float(fps)

        # Define Time to find the new frame location based on the
        # combination of the current frame, the number of frames to
        # advance, and the fps setting of the document
        c4dTime = c4d.BaseTime(frame, fps)

        # Move the playhead to the newly referenced location in the
        # timeline
        doc.SetTime(c4dTime)

        # DO NOT FUCKING TOUCH THIS - this command is like the super version of
        # c4d.EventAdd() and the timeline won't move unless you put this
        # command here
        #
        # Thank god for this explanation:
        # https://www.youtube.com/watch?v=dQw4w9WgXcQ
        # (https://www.c4dcafe.com/ipb/forums/topic/95496-zoetrope-with-python-script/)
        doc.ExecutePasses(None, True, True, True, 0)

        cubeSatPos = {}
        cubeSatRot = {}

        for i, cubeSatObj in enumerate(cubesats):

            rot = []

            # [cm]
            currPos = cubeSatObj.GetAbsPos()
            pos = c4dToVCF(currPos)

            # HPB Euler Angles
            # [rad]
            currRot = cubeSatObj.GetAbsRot()
            rot.append(currRot[0])
            rot.append(currRot[1])
            rot.append(currRot[2])
            q = c4d.Quaternion()
            q.SetHPB(currRot)

            idx = 'launch_num_' + str(i) + '__' + cubeSatObj.GetName()
            cubeSatPos[idx] = pos
            cubeSatRot[idx] = rot

        # the actual output data struct should have a field for the time step,
        # and the position vectors for each cubesat at each frame
        cubeSatStates.append({'t': currTime,
                              'pos': cubeSatPos})

    # build the simulation case directory and write the truth data and the
    # config file to json files
    outFPaths = []
    configName = settings['CONFIG_NAME']
    now = datetime.datetime.now()
    currTime = str(now.strftime('%H_%M_%S_%Y_%m_%d'))
    caseDirName = configName + '-' + currTime
    outputDir = os.path.join('..', '4_Simulation_Cases')

    configFName = configName + '.yaml'
    newConfigOutFPath = os.path.join(outputDir, caseDirName, configFName)

    outFName = configName + '_truth_data.json'
    outFPaths.append(os.path.join(outputDir, caseDirName, outFName))

    # make TOF and Optical data directories for saving data
    os.makedirs(os.path.join(outputDir, caseDirName, 'ToF_Data'))
    os.makedirs(os.path.join(outputDir, caseDirName, 'Optical_Data'))

    makeDirsFromFileNames(outFPaths)

    data = [cubeSatStates]

    for (currData, outFile) in zip(data, outFPaths):
        with open(outFile, 'w+') as outfile:
            json.dump(currData, outfile, indent=4)

    # copy the configuration file to the new output directory
    shutil.copy(settings['CONFIG_PATH'], newConfigOutFPath)


#
# @brief      transforms the position data from the c4d to the VCF coordinate
#             system
#
# @param      pos   The position in the c4d frame
#                   [cm]
#
# @return     the position in VCF [cm]
#
def c4dToVCF(posInC4D):

    # offset between the origins of the two frames, measured from c4d to VCF
    # [cm]
    c4dOriginToVCFOrigin = [0, 0, 0]

    x = posInC4D[0]
    y = posInC4D[1]
    z = posInC4D[2]

    v1 = x - c4dOriginToVCFOrigin[0]
    v2 = -y - c4dOriginToVCFOrigin[1]
    v3 = z - c4dOriginToVCFOrigin[2]

    return [v1, v2, v3]


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
    c4d.EventAdd()
