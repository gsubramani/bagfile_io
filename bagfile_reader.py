import rosbag
import rospy
import numpy as np
from scipy.signal import butter,filtfilt
from matplotlib import pyplot as plt
import shutil
import os


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

def write_to_bagfile(filepath,topicname,msgs,headerstamps,write_permissions,createbackup = True):

    # write_permissions -> 'a' -> append to bagfile ; 'w' -> write new bagfile

    if createbackup == True:
        shutil.copy2(filepath,"./" + filepath + "_backup.bag")

    elif type(createbackup) == type("string_type"):
        shutil.copy2(filepath, createbackup)

    with rosbag.Bag(filepath, write_permissions) as bag:
        if type(headerstamps[0]) == type(rospy.Time.from_sec(0)):
            for msg,stamp in zip(msgs,headerstamps):
                bag.write(topicname, msg, stamp)

        else:
            for msg,stamp in zip(msgs,headerstamps):
                bag.write(topicname, msg, rospy.Time.from_sec(stamp))

def remove_topic_from_bagfile(filepath,topic_to_remove):
    file_name = filepath.split('.bag')[0]
    file_name = file_name.split('/')[-1]
    temp_bag_path = "./" + file_name + "_backup.bag"
    shutil.copy2(filepath, temp_bag_path)
    os.remove(filepath)
    print "Please wait... Filtering bagfile"
    with rosbag.Bag(filepath, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(temp_bag_path).read_messages():
            if topic == topic_to_remove:
                pass
            else:
                outbag.write(topic, msg, t)
    os.remove(temp_bag_path)
    print "Done!"



class bagfile_reader():
    def __init__(self,bagfilepath):
        self.filepath = bagfilepath
        try:
            self.bag = rosbag.Bag(bagfilepath)
        except :
            # self.bag = rosbag.Bag(bagfilepath,'w')
            self.bag = None
            print "bag file not found!"
            return
        self.topics = set()

        self.data_dict = {}
        self.time_stamps_dict = {}
        for topic, _, _ in self.bag.read_messages():
            self.topics.add(topic)

        topic_list = list(self.topics)
        for topic in topic_list:
            self.data_dict[topic] = []
            self.time_stamps_dict[topic] = []

        for topic, msg, t in self.bag.read_messages():
            self.data_dict[topic].append(msg)
            self.time_stamps_dict[topic].append(t)

    def get_topic_msgs(self,topic_name):
        if topic_name in self.topics:
            msgs = np.array([element for element in self.data_dict[topic_name]])
            time_stamps = np.array(
                [(element).to_sec() for element in self.time_stamps_dict[topic_name]])  # - time_stamps_dict["/IMUData"][0]

        else:
            msgs = []
            time_stamps = []

        return msgs, time_stamps


    def get_topic_sample_rate(self,topic_name):
        time_stamps = np.array(
            [(element).to_sec() for element in self.time_stamps_dict[topic_name]])  # - time_stamps_dict["/IMUData"][0]

        return np.mean(np.diff(time_stamps))

    def get_possible_topic_values(self,topic_name):
        if topic_name in self.topics:
            return list(set(np.array([element for element in self.data_dict[topic_name]])))
        else:
            return []

    # Commented out because of a duplicate implementation above
    # def write_to_bagfile(self,topicname,msgs,headerstamps,createbackup = True):
    #
    #     if createbackup == True:
    #         shutil.copy2(self.filepath,"./" + self.filepath + "_backup.bag")
    #
    #     elif type(createbackup) == type("string_type"):
    #         shutil.copy2(self.filepath, createbackup)
    #
    #     with rosbag.Bag(self.filepath, 'a') as bag:
    #         if type(headerstamps[0]) == type(rospy.Time.from_sec(0)):
    #             for msg,stamp in zip(msgs,headerstamps):
    #                 bag.write(topicname, msg, stamp)
    #
    #         else:
    #             for msg,stamp in zip(msgs,headerstamps):
    #                 bag.write(topicname, msg, rospy.Time.from_sec(stamp))


if __name__ == "__main__":
# Example usage
    bf= bagfile_reader("Take1.bag")
    f1,ft1 = bf.get_topic_msgs("/labels")
    fo1, fto1 = bf.get_topic_msgs("/proxyforce1")
    plt.plot(ft1,[f.vector.x for f in f1])
    plt.plot(fto1, [f.vector.x + 1 for f in fo1])

    plt.show()

    # bf.write_to_bagfile("/proxyforce1",f1,ft1)

