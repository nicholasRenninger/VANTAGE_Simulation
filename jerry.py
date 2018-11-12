import c4d
# from c4d import documents, plugins
# Welcome to the world of Python

def main():
    camera = c4d.CallCommand(5103)
    camera()[c4d.CAMERA_FOCUS] = 36
    camera()[c4d.CAMERAOBJECT_APERTURE] = 36
    camera()[c4d.CAMERAOBJECT_FOV] = 0.927
    object()[c4d.CAMERAOBJECT_FOV_VERTICAL] = 0.548
    object()[c4d.CAMERAOBJECT_TARGETDISTANCE] = 2000
    object()[c4d.ID_BASEOBJECT_REL_POSITION,c4d.VECTOR_X] = 0
    object()[c4d.ID_BASEOBJECT_REL_POSITION,c4d.VECTOR_Y] = 0
    object()[c4d.ID_BASEOBJECT_REL_POSITION,c4d.VECTOR_Z] = 0
    object()[c4d.ID_BASEOBJECT_REL_ROTATION,c4d.VECTOR_X] = 0
    object()[c4d.ID_BASEOBJECT_REL_ROTATION,c4d.VECTOR_Y] = 0
    object()[c4d.ID_BASEOBJECT_REL_ROTATION,c4d.VECTOR_Z] = 0


if __name__=='__main__':
    main()
    c4d.EventAdd()