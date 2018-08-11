import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rosbag
from os.path import isfile, join, isdir
from os import listdir, mkdir
import numpy as np 
import json

np.set_printoptions(precision=3, suppress=True, threshold=10000)

# outvidfile = 'bag_save6.avi'
inputdir = '/datadrive/exp_bags/airsim_data'
cvbridge = CvBridge()
SaveDataToFile = True
Visualize = False
# image_size = (640,360)
# SaveVideo = False

topiclist = ['/cam_image/0/scene',
            # '/cam_image/0/depthperspective',
            '/cam_image/0/depthplanner',
            '/cam_image/0/camera_pose',
            '/cam_image/0/segmentation'
            ]

topicNum = len(topiclist)
topicDict = {topiclist[k]:k for k in range(topicNum)}


def msg_to_numpy(msgList):
    '''
    msgList:  3 ROS messages: Image, Image, PoseStramped
    image_np: 3-channel bgr image (height x width x 3)
    depth_np: 1-channel float32 numpy array (height x width)
    pose_np:  translation + orientation(quaternion) (1 x 7)
    '''
    image_np, depth_np_pers, depth_np_plan, pose_np, seg_np = [], [], [], [], []
    for ind, topicName in enumerate(topiclist):
        topicName = topicName.split('/')[-1]
        if topicName == 'scene':
            image_np = cvbridge.imgmsg_to_cv2(msgList[ind], "bgr8") 
        elif topicName == 'depthperspective':
            depth_np_pers = cvbridge.imgmsg_to_cv2(msgList[ind], "32FC1") * 100
        elif topicName == 'depthplanner':
            depth_np_plan = cvbridge.imgmsg_to_cv2(msgList[ind], "32FC1") * 100
        elif topicName == 'camera_pose':
            pos = msgList[ind].pose.position
            ori = msgList[ind].pose.orientation
            pose_np = np.array([pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])
        elif topicName == 'segmentation':
            seg_np = cvbridge.imgmsg_to_cv2(msgList[ind], "mono8")
        else:
            print ('Error Msg Type: %s' %(topicName))

    return image_np, depth_np_pers, depth_np_plan, pose_np, seg_np

def visualize(image_np, depth_np_pers, depth_np_plan, pose_np):
    '''
    visualize RGB image, and depth image
    print camera pose to screen
    '''
    image_vis = image_np
    depth_np_list = [depth_np_pers, depth_np_plan]
    for depth_np in depth_np_list:
        if len(depth_np) == 0:
            continue
        depth_np = np.clip(depth_np/100.0,0,1)
        depth_np = depth_np * 255.0
        depth_np = depth_np.astype(np.uint8)
        depth_np = np.tile(depth_np,(3,1,1)).transpose((1,2,0))
        if depth_np.shape[0] != image_np.shape[0]:
            depth_np = cv2.resize(depth_np, (image_np.shape[1],image_np.shape[0]))

        image_vis = np.concatenate((image_vis,depth_np),0)
    cv2.imshow('vis', image_vis)
    cv2.waitKey(0)
    print ('translation (%.4f, %.4f, %.4f), rotation (%.4f, %.4f, %.4f, %.4f)' % (pose_np[0], 
        pose_np[1], pose_np[2], pose_np[3], pose_np[4], pose_np[5], pose_np[6]))

def initializeDirs(bagfilename):
    outdir = join(inputdir, bagfilename)
    rgbdir = join(outdir, 'image')
    segdir = join(outdir, 'segmentation')
    depthdir_pers = join(outdir, 'depth_pers')
    depthdir_plan = join(outdir, 'depth_plan')
    posefilename = join(outdir,'pose.txt')
    posewonamefilename = join(outdir, 'pose_wo_name.npy')
    namefilefilename = join(outdir, 'pose_name.json')

    if isdir(outdir):
        print ('Directory Exists!! %s', bagfilename)
        return
    mkdir(outdir)
    mkdir(rgbdir)
    mkdir(segdir)
    mkdir(depthdir_pers)
    mkdir(depthdir_plan)
    posefile = open(posefilename, 'w')
    # posefile_wo_name = open(posewonamefilename,'w')
    namefile = open(namefilefilename,'w')
    return rgbdir, depthdir_pers, depthdir_plan, segdir, posefile, posewonamefilename, namefile


def save_to_file(image_np, seg_np, depth_np_pers, depth_np_plan, pose_np, ind, timestamp, rgbdir, segdir, depthdir_pers, depthdir_plan):
    timestr = str(timestamp/1000000%1000000)
    timestr = '0'*(6-len(timestr))+timestr
    imgname = '0'*(6-len(str(ind)))+str(ind)+'_'+timestr
    if len(image_np>0):
        cv2.imwrite(join(rgbdir, imgname+'_rgb.png'), image_np)
    if len(depth_np_pers)>0: 
        np.save(join(depthdir_pers,imgname+'_depth.npy'), depth_np_pers)
    if len(depthdir_pers)>0:
        np.save(join(depthdir_plan,imgname+'_depth.npy'), depth_np_plan)
    if len(seg_np)>0:
        np.save(join(segdir, imgname+'_seg.npy'), seg_np)
    # namelist = [] 
    posefile.write(imgname+' ')
    for val in pose_np:
        # namelist.append(str(val))
        posefile.write(str(val)+' ')
    posefile.write('\n')

    return imgname

# the messages returned by airsim is not well synchronized
# checkTimeSync will throw away unsynchronized frames
def checkTimeSync(timeList):
    lastt = timeList[0]
    print np.array(timeList)%1000000000, 
    for t in timeList:
        if abs(t-lastt)>10000000: #t!=lastt:
            print 'False'
            return False
        lastt=t
    print 'true'
    # raw_input()
    return True

# save multiple bagfiles in one folder into one single video
# if SaveVideo:
#     outvidfile = 'bag_save_gascola.avi'
#     fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fout=cv2.VideoWriter(outvidfile, fourcc, 30.0, image_size)
#     print outvidfile


for filename in listdir(inputdir):
    # filename = 'data_collection_blockworld_simple.bag' # debug
    filepathname = join(inputdir, filename)
    if (not isfile(filepathname)) or (not filename[-3:]=='bag'):
        continue

    bag = rosbag.Bag(filepathname, 'r')
    topics = bag.read_messages(topics=topiclist)

    if SaveDataToFile:
        rgbdir, depthdir_pers, depthdir_plan, segdir, posefile, posefile_wo_name, namefile = initializeDirs(bagfilename = filename[:-4])

    print (filename)
    syncTimeList = np.zeros(topicNum, dtype=np.int64)
    msgList = [0] * topicNum
    ind, count = 0, 0

    namelist = []
    poselist = []
    for topic, msg, t in topics:
        # import ipdb; ipdb.set_trace()
        # print topic, t.to_nsec()%1000000000, msg.header.stamp.to_nsec()%1000000000
        topicInd = topicDict[topic]
        syncTimeList[topicInd] = msg.header.stamp.to_nsec() #t.to_nsec() #
        msgList[topicInd] = msg

        if checkTimeSync(syncTimeList):
            rgb_np, depth_np_pers, depth_np_plan, pose_np, seg_np = msg_to_numpy(msgList)
            if Visualize:
                visualize(rgb_np, depth_np_pers, depth_np_plan, pose_np)
            if SaveDataToFile:
                imgname = save_to_file(rgb_np, seg_np, depth_np_pers, depth_np_plan, pose_np, ind, syncTimeList[0],rgbdir, segdir, depthdir_pers, depthdir_plan)
                namelist.append(imgname)
                poselist.append(pose_np)
            ind += 1

            if ind%1000==0:
                print ('   %d data...' % (ind))
            # if SaveVideo:
            #     fout.write(image_np)

        count += 1

    # import ipdb; ipdb.set_trace()
    if SaveDataToFile:
        # save pose and posename
        json.dump({'pose_name': namelist}, namefile)
        np.save(posefile_wo_name, np.array(poselist))

    print ('   %d data saved...' % (ind))
    bag.close()
    if SaveDataToFile:
        posefile.close()
        namefile.close()

    # break # debug
# if SaveVideo:
#     fout.release()

