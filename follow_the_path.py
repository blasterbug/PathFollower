"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 204-09-11
    Follow the path below
"""
import httplib, json, time, sys
from math import sin, cos, pi, atan2, sqrt

MRDS_URL = 'localhost:50000'

HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

class UnexpectedResponse(Exception): pass

def postSpeed(angularSpeed,linearSpeed):
    """Sends a speed command to the MRDS server"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    params = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
    mrds.request('POST','/lokarria/differentialdrive',params,HEADERS)
    response = mrds.getresponse()
    status = response.status
    #response.close()
    if status == 204:
        return response
    else:
        raise UnexpectedResponse(response)

def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData)
    else:
        return response
    
def getLaserAngles():
    """Requests the current laser properties from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/properties')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        properties = json.loads(laserData)
        beamCount = int((properties['EndAngle']-properties['StartAngle'])/properties['AngleIncrement'])
        a = properties['StartAngle']#+properties['AngleIncrement']
        angles = []
        while a <= properties['EndAngle']:
            angles.append(a)
            a+=pi/180 #properties['AngleIncrement']
        #angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
        return angles
    else:
        raise UnexpectedResponse(response)

def getPose():
    """Reads the current position and orientation from the MRDS"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/localization')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData)
    else:
        return UnexpectedResponse(response)

def bearing(q):
    return rotate(q,{'X':1.0,'Y':0.0,"Z":0.0})

def rotate(q,v):
    return vector(qmult(qmult(q,quaternion(v)),conjugate(q)))

def quaternion(v):
    q=v.copy()
    q['W']=0.0;
    return q

def vector(q):
    v={}
    v["X"]=q["X"]
    v["Y"]=q["Y"]
    v["Z"]=q["Z"]
    return v

def conjugate(q):
    qc=q.copy()
    qc["X"]=-q["X"]
    qc["Y"]=-q["Y"]
    qc["Z"]=-q["Z"]
    return qc

def qmult(q1,q2):
    q={}
    q["W"]=q1["W"]*q2["W"]-q1["X"]*q2["X"]-q1["Y"]*q2["Y"]-q1["Z"]*q2["Z"]
    q["X"]=q1["W"]*q2["X"]+q1["X"]*q2["W"]+q1["Y"]*q2["Z"]-q1["Z"]*q2["Y"]
    q["Y"]=q1["W"]*q2["Y"]-q1["X"]*q2["Z"]+q1["Y"]*q2["W"]+q1["Z"]*q2["X"]
    q["Z"]=q1["W"]*q2["Z"]+q1["X"]*q2["Y"]-q1["Y"]*q2["X"]+q1["Z"]*q2["W"]
    return q

# stands instead of getHeading()
def getBearing():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])
    
    
"""
    Follow a given path in a JSON file and command a robot throught MRDS to follow it.
    Author: Benjamin Sientzoff (ens15bsf@cs.umu.se)
"""

class FlippedOver(Exception): pass

lookAheadDistance = 1.2
thresholdPrecision = 0.4

"""
  get robot inclination
"""
def getInclino():
    """Reads the current position and orientation from the MRDS"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/inclinometer')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData)
    else:
        return UnexpectedResponse(response)
    
"""
  Compute the distqnce betwwen two points
  @param pointA coordinates of the first point
  @param pointB coordinates of the second point
  @return distance between point A and B
"""
def computeDistance( pointA, pointB ) :
    return sqrt( pow( pointB['X'] - pointA['X'], 2 ) + pow( pointB['Y'] - pointA['Y'], 2 ) )

"""
  Compute robot commands for the next path point
  use 'follow the carrot' algorithm
  @param carrot Point to reach
  @param robotPose Robot collection storing robot position and orientation
  @return speeds of the robots in a dictionnary
"""
def speedsToReach( carrot, robotPose ) :
    # robot angle
    angleRobot = atan2( getBearing()['Y'], getBearing()['X'] )
    # compute distance between the robot and the carrot
    distanceCarrot = computeDistance( carrot, robotPose['Position'] )
    # print "distance: ", distanceCarrot
    # compute the angle to the carrot
    angleCarrot = atan2( carrot['Y'] - robotPose['Position']['Y'], carrot['X'] - robotPose['Position']['X'] )
    angleToCarrot = angleCarrot - angleRobot
    if -pi > angleToCarrot :
       angleToCarrot = angleToCarrot + ( 2 * pi )
    # print "angle: ", angleToCarrot
    # compute angular speed
    speeds = {}
    speeds['angular'] = ( angleToCarrot ) / 1
    speeds['linear'] = ( speeds['angular'] * ( distanceCarrot / ( 2 * sin( angleToCarrot ) ) ) )
    if 0.4 < speeds['linear'] :
        speeds['linear'] = 0.4
    if -0.4 > speeds['linear'] :
        speeds['linear'] = -0.4
    return speeds

if __name__ == '__main__':
    # get path file
    try:
        # load the path
        path = json.load( open( sys.argv[1] ) )
        #print "Path :", json.dumps( path, sort_keys=True, indent=2, separators=( ',', ': ' ) )
        rbtPose = getPose()['Pose']
        # start the timer
        startTime = time.time()
        # for each point of the path
        for point in path :
            # extrait point coordinates
            nextPosition = point['Pose']['Position']
            print "Goal position: ", json.dumps( nextPosition, sort_keys=True, indent=2, separators=( ',', ': ' ) )
            # get robot position
            rbtPose = getPose()['Pose']
            print "Robot position :", json.dumps( rbtPose['Position'], sort_keys=True, indent=2, separators=( ',', ': ' ) )
            # if the next point to reach is quite away
            if lookAheadDistance < computeDistance( rbtPose['Position'], nextPosition ):
                # compute speed
                speeds = speedsToReach( nextPosition, rbtPose )
                print "speeds :", json.dumps( speeds, sort_keys=True, indent=2, separators=( ',', ': ' ) )
                # send the speed to the robot
                postSpeed( speeds['angular'], speeds['linear'] )
                # while the point is not reached
                while thresholdPrecision < computeDistance( rbtPose['Position'], nextPosition ):
                    rbtPose = getPose()['Pose']
                    # compute speed
                    speeds = speedsToReach( nextPosition, rbtPose )
                    print "speeds :", json.dumps( speeds, sort_keys=True, indent=2, separators=( ',', ': ' ) )
                    # send the speed to the robot
                    postSpeed( speeds['angular'], speeds['linear'] )
                    time.sleep( 0.5 )
            # detect robot flipped over, does it work ?
            if rbtPose['Position']['Z'] < 0 :
                raise FlippedOver()
        # stop the robot
        postSpeed( 0, 0 )
        endTime = time.time()
        print endTime - startTime, "Seconds to follow the path"
    except FlippedOver :
        postSpeed( 0, 0 ) # stop the robot
        # robot flipped over
        sys.stderr.write( "Robot flipped over" )    
    # catch except and display an error message if : 
    except IndexError :
        # no given path file
        sys.stderr.write( "Give a filename to load a path" )
    except err :
        sys.stderr.write( "?!" )
