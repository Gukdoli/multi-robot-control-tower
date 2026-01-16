// Multi-robot configuration for Control Tower

const DEFAULT_ROBOTS = [
  {
    id: 'robot1',
    name: 'Robot 1',
    namespace: '/robot1',
    color: '#FF6B6B',
    domain: 1
  },
  {
    id: 'robot2',
    name: 'Robot 2',
    namespace: '/robot2',
    color: '#4ECDC4',
    domain: 2
  },
  {
    id: 'robot3',
    name: 'Robot 3',
    namespace: '/robot3',
    color: '#45B7D1',
    domain: 3
  }
];

export const loadRobots = () => {
  const saved = localStorage.getItem('robots');
  return saved ? JSON.parse(saved) : DEFAULT_ROBOTS;
};

export const saveRobots = (robots) => {
  localStorage.setItem('robots', JSON.stringify(robots));
};

export const addRobot = (robots, newRobot) => {
  const updated = [...robots, newRobot];
  saveRobots(updated);
  return updated;
};

export const updateRobot = (robots, robotId, updates) => {
  const updated = robots.map(r => r.id === robotId ? { ...r, ...updates } : r);
  saveRobots(updated);
  return updated;
};

export const deleteRobot = (robots, robotId) => {
  const updated = robots.filter(r => r.id !== robotId);
  saveRobots(updated);
  return updated;
};

export const getNamespacedTopic = (namespace, topic) => {
  if (namespace.startsWith('/')) {
    return `${namespace}${topic}`;
  }
  return `/${namespace}${topic}`;
};
