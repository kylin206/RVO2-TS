import KdTree from '../KdTree';
import Simulator from '../Simulator';
import Vector2D from '../Vector2D';
import Agent from '../Agent';

describe('KdTree', () => {
  let kdTree: KdTree;
  let simulator: Simulator;

  beforeEach(() => {
    simulator = new Simulator();
    kdTree = new KdTree();
    kdTree.simulator = simulator;
  });

  describe('agent tree operations', () => {
    beforeEach(() => {
      simulator.setAgentDefaults(
        5,   // neighborDist 减小邻居距离以确保检测
        10,  // maxNeighbors
        10,  // timeHorizon
        5,   // timeHorizonObst
        1,   // radius
        2    // maxSpeed
      );

      // 添加更近的agents
      simulator.addAgent(new Vector2D(0, 0));
      simulator.addAgent(new Vector2D(1, 0));  // 距离为1
      simulator.addAgent(new Vector2D(0, 1));  // 距离为1
    });

    it('should build agent tree', () => {
      expect(simulator.getNumAgents()).toBe(3);
      
      kdTree.buildAgentTree();

      const testAgent = new Agent();
      testAgent.position = new Vector2D(0.5, 0.5);
      testAgent.maxNeighbors = 10;
      testAgent.neighborDist = 5;
      testAgent.radius = 1;
      testAgent.simulator = simulator;

      kdTree.computeAgentNeighbors(testAgent, 9); // 3^2
      expect(testAgent.agentNeighbors.length).toBeGreaterThan(0);
    });
  });

  describe('obstacle tree operations', () => {
    beforeEach(() => {
      // 创建一个简单的正方形障碍物，确保testAgent位于其中一条边的正下方，确保障碍物为逆时针
      const vertices = [
        new Vector2D(-1, -1), // 左下
        new Vector2D(1, -1),  // 右下
        new Vector2D(1, 1),   // 右上
        new Vector2D(-1, 1)   // 左上
      ];
      
      simulator.addObstacle(vertices);
      simulator.processObstacles();

      // 验证障碍物链表的正确性
      const obstacles = simulator.getObstacles();
      expect(obstacles.length).toBe(4);
      
      // 验证循环链接
      for (let i = 0; i < obstacles.length; i++) {
        const curr = obstacles[i];
        const next = obstacles[(i + 1) % obstacles.length];
        const prev = obstacles[(i - 1 + obstacles.length) % obstacles.length];
        
        expect(curr.next).toBe(next);
        expect(curr.previous).toBe(prev);
        expect(curr.point).toBeDefined();
      }
    });

    it('should build obstacle tree and detect neighbors', () => {
      kdTree.buildObstacleTree();
      
      // 创建一个位于正方形底边正下方的测试agent
      const testAgent = new Agent();
      testAgent.position = new Vector2D(0, -2);  // 位于底边正下方1个单位
      testAgent.radius = 0.1;
      testAgent.maxNeighbors = 10;
      testAgent.simulator = simulator;
      
      // 搜索范围应该足以覆盖到正方形的底边
      const rangeSq = 4; // 2^2
      kdTree.computeObstacleNeighbors(testAgent, rangeSq);
      
      // 应该至少检测到底边作为障碍物
      expect(testAgent.obstaclNeighbors.length).toBeGreaterThan(0);

      // 验证找到的障碍物
      if (testAgent.obstaclNeighbors.length > 0) {
        const nearest = testAgent.obstaclNeighbors[0].value;
        expect(nearest.point.y).toBeCloseTo(-1); // 应该是底边上的某个点
      }
    });

    it('should handle visibility queries', () => {
      kdTree.buildObstacleTree();
      
      // 测试从下方穿过正方形的射线
      const p1 = new Vector2D(0, -2);
      const p2 = new Vector2D(0, 2);
      const visible = kdTree.queryVisibility(p1, p2, 0.1);
      expect(visible).toBe(false); // 应该被正方形阻挡
      
      // 测试在正方形外部的射线
      const p3 = new Vector2D(-2, 0);
      const p4 = new Vector2D(-1.5, 0);  // 正方形左侧
      const visibleOutside = kdTree.queryVisibility(p3, p4, 0.1);
      expect(visibleOutside).toBe(true);
    });
  });
});