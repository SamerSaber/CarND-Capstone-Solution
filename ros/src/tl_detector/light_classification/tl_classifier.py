from styx_msgs.msg import TrafficLight

#implement traffic light classification.
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    # Take a camera image as input and return an ID corresponding to the color state of the traffic light in the image.
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
