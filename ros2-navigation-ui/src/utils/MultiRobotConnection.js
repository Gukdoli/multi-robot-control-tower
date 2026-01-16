import ROSLIB from 'roslib';
import { getNamespacedTopic } from '../config/multiRobotConfig';

class MultiRobotConnection {
  constructor(url) {
    this.ros = new ROSLIB.Ros({
      url: url
    });
    this.robotSubscriptions = {};
    this.robotCallbacks = {};
  }

  onConnect(callback) {
    this.ros.on('connection', callback);
  }

  onError(callback) {
    this.ros.on('error', callback);
  }

  onClose(callback) {
    this.ros.on('close', callback);
  }

  close() {
    Object.keys(this.robotSubscriptions).forEach(robotId => {
      if (this.robotSubscriptions[robotId].position) {
        this.robotSubscriptions[robotId].position.unsubscribe();
      }
      if (this.robotSubscriptions[robotId].map) {
        this.robotSubscriptions[robotId].map.unsubscribe();
      }
    });
    this.ros.close();
  }

  sendGoal(robotNamespace, x, y) {
    const goalTopicName = getNamespacedTopic(robotNamespace, '/goal_pose');

    const goalTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: goalTopicName,
      messageType: 'geometry_msgs/PoseStamped'
    });

    const goalMsg = new ROSLIB.Message({
      header: {
        frame_id: 'map',
        stamp: {
          sec: Math.floor(Date.now() / 1000),
          nanosec: (Date.now() % 1000) * 1000000
        }
      },
      pose: {
        position: { x: x, y: y, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
      }
    });

    goalTopic.publish(goalMsg);
    console.log(`[${robotNamespace}] Goal published:`, { x, y });
  }

  subscribeToRobotPosition(robotId, robotNamespace, callback) {
    if (!this.robotCallbacks[robotId]) {
      this.robotCallbacks[robotId] = {};
    }
    this.robotCallbacks[robotId].position = callback;

    this.ros.getTopics((result) => {
      const topics = result.topics;
      const namespacedOdometry = getNamespacedTopic(robotNamespace, '/odometry');
      const namespacedOdom = getNamespacedTopic(robotNamespace, '/odom');

      if (topics.includes(namespacedOdometry)) {
        this.setupOdometrySubscription(robotId, namespacedOdometry, 'nav_msgs/Odometry');
      } else {
        this.setupOdometrySubscription(robotId, namespacedOdom, 'nav_msgs/Odometry');
      }
    });
  }

  setupOdometrySubscription(robotId, topicName, messageType) {
    if (!this.robotSubscriptions[robotId]) {
      this.robotSubscriptions[robotId] = {};
    }

    const subscription = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType
    });

    subscription.subscribe((message) => {
      const pose = message.pose.pose;
      const position = {
        x: pose.position.x,
        y: pose.position.y,
        z: pose.position.z
      };

      if (this.robotCallbacks[robotId]?.position) {
        this.robotCallbacks[robotId].position(position);
      }
    });

    this.robotSubscriptions[robotId].position = subscription;
  }

  subscribeToMap(robotId, robotNamespace, callback) {
    if (!this.robotCallbacks[robotId]) {
      this.robotCallbacks[robotId] = {};
    }
    this.robotCallbacks[robotId].map = callback;

    const mapTopicName = getNamespacedTopic(robotNamespace, '/map');

    if (!this.robotSubscriptions[robotId]) {
      this.robotSubscriptions[robotId] = {};
    }

    const mapSubscription = new ROSLIB.Topic({
      ros: this.ros,
      name: mapTopicName,
      messageType: 'nav_msgs/OccupancyGrid'
    });

    mapSubscription.subscribe((message) => {
      if (this.robotCallbacks[robotId]?.map) {
        this.robotCallbacks[robotId].map(message);
      }
    });

    this.robotSubscriptions[robotId].map = mapSubscription;
  }

  unsubscribeRobot(robotId) {
    if (this.robotSubscriptions[robotId]) {
      if (this.robotSubscriptions[robotId].position) {
        this.robotSubscriptions[robotId].position.unsubscribe();
      }
      if (this.robotSubscriptions[robotId].map) {
        this.robotSubscriptions[robotId].map.unsubscribe();
      }
      delete this.robotSubscriptions[robotId];
      delete this.robotCallbacks[robotId];
    }
  }

  unsubscribeRobotMap(robotId) {
    if (this.robotSubscriptions[robotId]?.map) {
      this.robotSubscriptions[robotId].map.unsubscribe();
      delete this.robotSubscriptions[robotId].map;
      delete this.robotCallbacks[robotId].map;
    }
  }
}

export default MultiRobotConnection;
