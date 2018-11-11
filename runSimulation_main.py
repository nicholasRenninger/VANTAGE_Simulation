import c4d
import yaml
import os


__author__ = "Nicholas Renninger, Jerry Wang"
__copyright__ = "'Copyright' 2018, VANTAGE"
__credits__ = ["Long Dong Silver"]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Nicholas Renninger"
__email__ = "nicholas.renninger@colorado.edu"
__status__ = "Development"


def main():

    settingsFile = os.path.join('config', 'config_simulation_template.yaml')

    # this is the current c4d session we need to modify
    doc = c4d.documents.GetActiveDocument()

    # all settings are defined in settingsFile
    settings = simulationSetup(settingsFile, doc)

    # load in the deployer model and set it up
    des_tube_origin = setup_deployer(settings, doc)[1]

    # load in the all of the cubesats, position them, and prescribe their
    # motion
    setup_cubesats(settings, des_tube_origin, doc)


#
# @brief      Loads in YAML config file settings and sets all overall
#             simulation settings from YAML config
#
#             Example settings: camera FPS, total simulation time, lighting,
#                               etc.
#
# @param      settingsFile  The settings YAML relative filepath
# @param      doc           current C4D document to setup simulation in
#
# @return     Settings dict from YAML config
#
def simulationSetup(settingsFile, doc):

    # Read YAML settings file
    with open(settingsFile, 'r') as stream:
        settings = yaml.load(stream)

    # ====================================
    # set the overall simulation parameters
    # ====================================
    fps = settings['OPTICAL_CAMERA_FPS']
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

    # keep the max simulation time for later
    settings['MAX_TIME'] = maxTimeInS

    return settings


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
    gDrive = settings['G_DRIVE_PATH']
    fullFile = os.path.join(gDrive, dep)

    # Import Deployer STL
    c4d.documents.MergeDocument(doc, fullFile, 1)
    deployerObj = doc.GetActiveObject()

    # Position and Rotate the Deployer so the global origin is at VANTAGE's
    # origin
    VANTAGE_origin_pos = settings['VANTAGE_POS']
    VANTAGE_origin_rot = settings['VANTAGE_ROT']
    pos_rot_obj(deployerObj, VANTAGE_origin_pos, VANTAGE_origin_rot)

    c4d.EventAdd()

    # save deployer geometry variables
    mu = settings['MU']
    beta = settings['BETA']
    sigma = settings['SIGMA']
    psi = settings['PSI']
    gamma = settings['GAMMA']
    L = settings['L']

    # define the origin of the desired tube relative to the deployer origin
    desired_tube = settings['DESIRED_DEPLOYMENT_TUBE']
    tube_coeffs = settings['TUBE_ORIGIN_COEFFS'][desired_tube - 1]
    x_coeff = tube_coeffs['x']
    y_coeff = tube_coeffs['y']
    z_coeff = tube_coeffs['z']

    des_tube_x = x_coeff[0] * sigma + x_coeff[1] * psi + x_coeff[2] * gamma
    des_tube_y = y_coeff[0] * mu + y_coeff[1] * beta
    des_tube_z = z_coeff[0] * L

    # now, translate this tube origin to its correct place in the global
    # coordinate system by adding the translation that moves the deployer
    # origin to the global origin
    des_tube_x += VANTAGE_origin_pos[0]
    des_tube_y += VANTAGE_origin_pos[1]
    des_tube_z += VANTAGE_origin_pos[2]

    # package output
    des_tube_origin = [des_tube_x, des_tube_y, des_tube_z]

    # Kobe
    print "Loaded Deployer Model"

    return deployerObj, des_tube_origin


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
    gDrive = settings['G_DRIVE_PATH']

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
        fullFile = os.path.join(gDrive, cubesat)

        # get length of the CubeSat Model
        # [cm]
        cubesat_length = CubeSatSize * u_size + 2 * cubeSat_tab_size

        c4d.documents.MergeDocument(doc, fullFile, 1)
        CubeSatObj = doc.GetActiveObject()
        objs.append(CubeSatObj)

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

        rotStart = rot

        animateCubeSat(CubeSatObj, posStart, rotStart,
                       v, omega, cubesat_length,
                       settings, des_tube_origin, doc)

    return objs


#
# @brief      moves and rotates the C4D obj by pos and rot respectively
#
# @param      obj   The C4D object to manipulate
# @param      pos   A list of positions in global coords to move obj by
#                   [cm]
# @param      rot   A list of euler angles rotations to rotate obj by
#                   [obj]
#
# @return     None
#
def pos_rot_obj(obj, pos, rot):

    obj()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_X] = rot[0]
    obj()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_Y] = rot[1]
    obj()[c4d.ID_BASEOBJECT_REL_ROTATION, c4d.VECTOR_Z] = rot[2]
    obj()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_X] = pos[0]
    obj()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_Y] = pos[1]
    obj()[c4d.ID_BASEOBJECT_REL_POSITION, c4d.VECTOR_Z] = pos[2]


def animateCubeSat(CubeSatObj, posStart, rotStart,
                   cubeSatLinVel, cubeSatRotVel, cubesat_length, settings,
                   des_tube_origin, doc):

    # [cm]
    L = settings['L']

    # maximum distance all of the CubeSats should end up at
    # [cm]
    max_dist = settings['MAX_RANGE']

    # time until the cubesats reach MAX_DISTANCE
    # [s]
    max_time = settings['MAX_TIME']

    fps = doc.GetFps()

    # getting the starting and ending frames for all of the animations
    (linVelZStartFrame,
     rotAndLinXYVelStartFrame,
     endFrame) = getAnimationFrameNumbers(L, cubesat_length,
                                          posStart, max_dist,
                                          des_tube_origin,
                                          fps, cubeSatLinVel[2])

    # get the ending position and velocity of the CubeSats
    posEnd, rotEnd = getLastPosAndRot(max_time, fps, cubeSatLinVel,
                                      cubeSatRotVel, L, posStart,
                                      rotAndLinXYVelStartFrame, endFrame)

    # get animation tracks for position and rotation
    obj_type_id = c4d.ID_BASEOBJECT_POSITION
    xtrack = getXYZtrack(CubeSatObj, obj_type_id, c4d.VECTOR_X, doc)
    ytrack = getXYZtrack(CubeSatObj, obj_type_id, c4d.VECTOR_Y, doc)
    ztrack = getXYZtrack(CubeSatObj, obj_type_id, c4d.VECTOR_Z, doc)

    obj_type_id = c4d.ID_BASEOBJECT_ROTATION
    wxtrack = getXYZtrack(CubeSatObj, obj_type_id, c4d.VECTOR_X, doc)
    wytrack = getXYZtrack(CubeSatObj, obj_type_id, c4d.VECTOR_Y, doc)
    wztrack = getXYZtrack(CubeSatObj, obj_type_id, c4d.VECTOR_Z, doc)

    # start recording addition of animations to timeline
    CubeSatObj.InsertTrackSorted(xtrack)
    CubeSatObj.InsertTrackSorted(ytrack)
    CubeSatObj.InsertTrackSorted(ztrack)

    # add position animation - linear variation of position from start to end
    # frames
    addValueAtFrame(xtrack, ytrack, ztrack, rotAndLinXYVelStartFrame,
                    linVelZStartFrame, fps, posStart)
    addValueAtFrame(xtrack, ytrack, ztrack, endFrame, endFrame, fps, posEnd)

    # add roatation animation - linear variation of rotation from start to end
    # frames
    addValueAtFrame(wxtrack, wytrack, wztrack, rotAndLinXYVelStartFrame,
                    rotAndLinXYVelStartFrame, fps, rotStart)
    addValueAtFrame(wxtrack, wytrack, wztrack, endFrame, endFrame, fps, rotEnd)

    # Goto Start
    c4d.CallCommand(12501)


def getXYZtrack(obj, trackID, channelID, doc):

    # Find the Track
    param = c4d.DescID(c4d.DescLevel(trackID, c4d.DTYPE_VECTOR, 0),
                       c4d.DescLevel(channelID, c4d.DTYPE_REAL, 0)
                       )
    track = obj.FindCTrack(param)

    # Create if no track found
    if not track:
        track = c4d.CTrack(obj, param)
        doc.AddUndo(c4d.UNDOTYPE_NEW, track)
        obj.InsertTrackSorted(track)

    return track


def addValueAtFrame(xtrack, ytrack, ztrack, XYStartFrame,
                    ZStartFrame, fps, startVec):

    # Add a key to each CubeSat linear position tracks
    xtrack = xtrack.GetCurve()
    xkey = xtrack.AddKey(c4d.BaseTime(XYStartFrame, fps))['key']
    xkey.SetValue(xtrack, startVec[0])

    # set the track position variation to linear
    c4d.CallCommand(465001092)

    ytrack = ytrack.GetCurve()
    ykey = ytrack.AddKey(c4d.BaseTime(XYStartFrame, fps))['key']
    ykey.SetValue(ytrack, startVec[1])

    # set the track position variation to linear
    c4d.CallCommand(465001092)

    ztrack = ztrack.GetCurve()
    zkey = ztrack.AddKey(c4d.BaseTime(ZStartFrame, fps))['key']
    zkey.SetValue(ztrack, startVec[2])

    # set the track position variation to linear
    c4d.CallCommand(465001092)


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
# @param      L               Length of the Deployer
#                             [cm]
# @param      cubesat_length  The cubesat length
#                             [cm]
# @param      posStart        The starting centroid position of the cubesat
#                             [cm]
# @param      posEnd          The ending centroid position of the cubesat
#                             [cm]
# @param      fps             The fps of the simulation
# @param      v_z             The main cubesat velocity component
#
# @return     [start frame for lin. z velocity animation,
#              start frame for rot. and lin. x-y velocity animation,
#              start frame for both lin. & rot velocity animation]
#
def getAnimationFrameNumbers(L, cubesat_length, posStart, maxDist,
                             des_tube_origin, fps, v_z):

    # need to compute frame numbers for where to apply the animation starts and
    # stops for linear and rotational animations

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


if __name__ == '__main__':
    # clear the screen if running as main
    print ('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n' +
           '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n' +
           '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n' +
           '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
    main()
    c4d.EventAdd()
