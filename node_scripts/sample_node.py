#!/usr/bin/env python

import rospy
from sound_play.libsoundplay import SoundClient

from actionlib_msgs.msg import GoalStatus
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sound_play.msg import SoundRequestAction


class  SampleNode(object):
    def __init__(self):
        self.sub = rospy.Subscriber(
            '~input',
            SpeechRecognitionCandidates,
            self._sub_cb,
            queue_size=1
        )
        self.sound_client = SoundClient(
            blocking=True,
            sound_action='robotsound_jp',
            sound_topic='robotsound_jp',
        )
        self.wait_duration = rospy.get_param('~wait_duration', 5)
        self.conf_thresh = rospy.get_param('~confidence_threshold', 0.95)
        self.prev_time = None

    def _sub_cb(self, msg):
        speech_text = msg.transcript[0]

        if len(msg.confidence) > 0:
            speech_conf = msg.confidence[0]
            if speech_conf < self.conf_thresh:
                return

        rospy.loginfo('speech    : {}'.format(speech_text))
        rospy.loginfo('confidence: {}'.format(msg.confidence))
        for st in self.sound_client.actionclient.action_client.last_status_msg.status_list:
            if st.status == GoalStatus.ACTIVE:
                return

        current_time = rospy.Time.now()
        if (self.prev_time is not None
                and current_time < (self.prev_time + rospy.Duration(self.wait_duration))):
            return

        self.sound_client.say(
            speech_text,
        )
        self.prev_time = current_time


if __name__ == '__main__':
    rospy.init_node('sample_node')
    node = SampleNode()
    rospy.spin()
