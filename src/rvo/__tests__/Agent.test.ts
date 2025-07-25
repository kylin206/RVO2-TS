import Agent from '../Agent';
import Simulator from '../Simulator';
import Vector2D from '../Vector2D';
import Line from '../Line';
import RVOMath from '../RVOMath';

describe('Agent', () => {
  let simulator: Simulator;
  let agent: Agent;

  beforeEach(() => {
    simulator = new Simulator();
    simulator.setAgentDefaults(
      5,   // neighborDist
      10,  // maxNeighbors
      10,  // timeHorizon
      5,   // timeHorizonObst
      1,   // radius
      2,   // maxSpeed
      0, 0 // default velocity
    );

    // 创建测试用的agent
    agent = new Agent();
    agent.simulator = simulator;
    agent.position = new Vector2D(0, 0);
    agent.velocity = new Vector2D(0, 0);
    agent['_newVelocity'] = new Vector2D(0, 0);
    agent.prefVelocity = new Vector2D(1, 0);
    agent.radius = 1;
    agent.maxSpeed = 2;
    agent.neighborDist = 5;
    agent.timeHorizon = 10;
    agent.timeHorizonObst = 5;
    agent.maxNeighbors = 10;

    // 将agent添加到simulator中
    simulator.agents.push(agent);
  });

  describe('basic properties', () => {
    it('should initialize with correct values', () => {
      expect(agent.position.x).toBe(0);
      expect(agent.position.y).toBe(0);
      expect(agent.velocity.x).toBe(0);
      expect(agent.velocity.y).toBe(0);
      expect(agent.prefVelocity.x).toBe(1);
      expect(agent.prefVelocity.y).toBe(0);
      expect(agent.radius).toBe(1);
      expect(agent.maxSpeed).toBe(2);
    });
  });

  describe('neighbor operations', () => {
    it('should insert agent neighbor within range', () => {
      const otherAgent = new Agent();
      otherAgent.position = new Vector2D(2, 0);
      agent.maxNeighbors = 5;
      
      agent.insertAgentNeighbor(otherAgent, 25);
      
      expect(agent.agentNeighbors.length).toBe(1);
      expect(agent.agentNeighbors[0].value).toBe(otherAgent);
    });

    it('should not insert agent neighbor outside range', () => {
      const otherAgent = new Agent();
      otherAgent.position = new Vector2D(6, 0);
      agent.maxNeighbors = 5;
      
      agent.insertAgentNeighbor(otherAgent, 25);
      
      expect(agent.agentNeighbors.length).toBe(0);
    });
  });

  describe('collision avoidance', () => {
    beforeEach(() => {
      // 将agent移动到更接近障碍物的位置
      agent.position = new Vector2D(0, -1.5);

      // 添加一个简单的水平线段障碍物
      const vertices = [
        new Vector2D(-2, 0),  // 左端点
        new Vector2D(2, 0),   // 右端点
      ];
      
      simulator.addObstacle(vertices);
      simulator.processObstacles();

      // 验证障碍物被正确添加
      const obstacles = simulator.getObstacles();
      expect(obstacles.length).toBe(2);
      expect(obstacles[0].next).toBe(obstacles[1]);
      expect(obstacles[1].next).toBe(obstacles[0]);

      // 添加另一个agent作为邻居
      const neighborAgent = new Agent();
      neighborAgent.position = new Vector2D(2, -1.5);
      neighborAgent.velocity = new Vector2D(-1, 0);
      neighborAgent.radius = 1;
      neighborAgent.simulator = simulator;
      simulator.agents.push(neighborAgent);

      // 重建KdTree
      simulator.kdTree.buildAgentTree();
      simulator.kdTree.buildObstacleTree();
    });

    it('should manage obstacle neighbors', () => {
      // 使用较大的检测范围
      agent.neighborDist = 10;
      agent.timeHorizonObst = 10;

      agent.computeNeighbors();

      // 打印调试信息
      console.log('Agent position:', agent.position);
      console.log('Obstacle neighbors:', agent.obstaclNeighbors.length);
      if (agent.obstaclNeighbors.length > 0) {
        console.log('First obstacle:', agent.obstaclNeighbors[0].value.point);
      }

      expect(agent.obstaclNeighbors.length).toBeGreaterThan(0);
    });

    it('should manage agent neighbors', () => {
      agent.computeNeighbors();
      expect(agent.agentNeighbors.length).toBeGreaterThan(0);
    });

    it('should compute velocity modifications', () => {
      const originalVelocity = agent.velocity.clone();
      agent.computeNeighbors();
      agent.computeNewVelocity();
      agent.update();
      
      // 速度应该被修改以避免碰撞
      expect(agent.velocity).not.toEqual(originalVelocity);
    });
  });

  describe('update operation', () => {
    it('should update position based on velocity', () => {
      agent.velocity = new Vector2D(1, 1);
      agent['_newVelocity'] = new Vector2D(1, 1);
      simulator.timeStep = 0.5;
      
      const oldPos = agent.position.clone();
      agent.update();
      
      expect(agent.position.x).toBeCloseTo(oldPos.x + 0.5);
      expect(agent.position.y).toBeCloseTo(oldPos.y + 0.5);
    });
  });
});